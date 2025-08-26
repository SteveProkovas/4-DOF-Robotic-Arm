#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import os
from datetime import datetime
import serial
import serial.tools.list_ports
import queue

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class SerialManager:
    def __init__(self, port=None, baud=115200, timeout=0.5):
        self.port = port
        self.baud = int(baud)
        self.timeout = timeout
        self.ser = None
        self._stop = threading.Event()
        self.reader_thread = None
        self.q = queue.Queue()

    def open(self):
        if not self.port:
            raise RuntimeError('port not set')
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(0.05)
        self._stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def close(self):
        self._stop.set()
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=0.5)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.readline()
            except Exception as e:
                self.q.put(('error', str(e)))
                break
            if not data:
                continue
            ts = datetime.utcnow().isoformat(sep=' ', timespec='milliseconds')
            try:
                text = data.decode('utf-8', errors='replace').rstrip('\r\n')
            except:
                text = repr(data)
            self.q.put(('line', f"{ts} {text}"))

    def write(self, data: bytes):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('serial not open')
        self.ser.write(data)

    def toggle_reset(self, rst_ms=100):
        if not self.ser:
            raise RuntimeError('serial not open')
        try:
            self.ser.dtr = False
            time.sleep(rst_ms/1000.0)
            self.ser.dtr = True
            time.sleep(0.02)
            self.q.put(('line', '[RESET] toggled DTR'))
        except Exception as e:
            self.q.put(('error', str(e)))

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('DSD-i1 Serial Console')
        self.geometry('900x600')
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        self.serial_mgr = SerialManager()
        self.log_path = None
        self.create_widgets()
        self.poll_queue()

    def create_widgets(self):
        top = ttk.Frame(self)
        top.pack(side='top', fill='x', padx=6, pady=6)

        ttk.Label(top, text='Port').pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, values=list_serial_ports(), width=12)
        self.port_combo.pack(side='left', padx=4)

        ttk.Button(top, text='Refresh', command=self.refresh_ports).pack(side='left', padx=4)

        ttk.Label(top, text='Baud').pack(side='left', padx=(8,0))
        self.baud_var = tk.StringVar(value='115200')
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side='left', padx=4)

        self.connect_btn = ttk.Button(top, text='Connect', command=self.toggle_connect)
        self.connect_btn.pack(side='left', padx=6)

        self.reset_btn = ttk.Button(top, text='Reset', command=self.reset_board)
        self.reset_btn.pack(side='left', padx=4)

        self.fpga_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(top, text='FPGA mode', variable=self.fpga_var).pack(side='left', padx=6)

        ttk.Label(top, text='Header').pack(side='left')
        self.header_var = tk.StringVar(value='0xA5')
        ttk.Entry(top, textvariable=self.header_var, width=6).pack(side='left', padx=4)

        ttk.Button(top, text='Send File', command=self.send_file_plain_dialog).pack(side='left', padx=6)
        ttk.Button(top, text='Send File (FPGA)', command=self.send_file_fpga_dialog).pack(side='left', padx=4)

        mid = ttk.Frame(self)
        mid.pack(side='top', fill='both', expand=True, padx=6, pady=6)

        self.text = tk.Text(mid, wrap='none')
        self.text.pack(side='left', fill='both', expand=True)
        self.text.config(state='disabled')

        vsb = ttk.Scrollbar(mid, orient='vertical', command=self.text.yview)
        vsb.pack(side='right', fill='y')
        self.text['yscrollcommand'] = vsb.set

        bottom = ttk.Frame(self)
        bottom.pack(side='bottom', fill='x', padx=6, pady=6)

        self.entry_var = tk.StringVar()
        self.entry = ttk.Entry(bottom, textvariable=self.entry_var)
        self.entry.pack(side='left', fill='x', expand=True, padx=(0,6))
        self.entry.bind('<Return>', self.on_send)

        ttk.Button(bottom, text='Send', command=self.on_send).pack(side='left')
        ttk.Button(bottom, text='Clear', command=self.clear_console).pack(side='left', padx=6)
        ttk.Button(bottom, text='Save Log', command=self.save_log).pack(side='left')

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo['values'] = ports

    def toggle_connect(self):
        if self.serial_mgr.ser is None:
            port = self.port_var.get() or None
            if not port:
                messagebox.showerror('Error', 'Select a serial port first')
                return
            try:
                baud = int(self.baud_var.get())
            except:
                messagebox.showerror('Error', 'Invalid baud')
                return
            self.serial_mgr.port = port
            self.serial_mgr.baud = baud
            try:
                self.serial_mgr.open()
            except Exception as e:
                messagebox.showerror('Error', f'Could not open port: {e}')
                return
            self.connect_btn.config(text='Disconnect')
            self.log_line(f'[+] Connected {port} @ {baud}')
        else:
            try:
                self.serial_mgr.close()
            except Exception:
                pass
            self.connect_btn.config(text='Connect')
            self.log_line('[+] Disconnected')

    def reset_board(self):
        try:
            self.serial_mgr.toggle_reset()
        except Exception as e:
            messagebox.showerror('Error', f'Reset failed: {e}')

    def on_send(self, event=None):
        text = self.entry_var.get()
        if not text:
            return
        tosend = text + '\n'
        try:
            if self.fpga_var.get():
                header = self.parse_header()
                frame = bytes([header]) + tosend.encode('utf-8', errors='replace')
                self.serial_mgr.write(frame)
            else:
                self.serial_mgr.write(tosend.encode('utf-8', errors='replace'))
            self.log_line(f'[TX] {text}')
        except Exception as e:
            messagebox.showerror('Error', f'Send failed: {e}')
        self.entry_var.set('')

    def parse_header(self):
        raw = self.header_var.get().strip()
        try:
            return int(raw, 0) & 0xFF
        except:
            return 0xA5

    def send_file_plain_dialog(self):
        path = filedialog.askopenfilename(title='Select file to send')
        if not path:
            return
        threading.Thread(target=self._send_file_plain, args=(path,), daemon=True).start()

    def _send_file_plain(self, path):
        try:
            size = os.path.getsize(path)
            sent = 0
            with open(path, 'rb') as f:
                while True:
                    chunk = f.read(1024)
                    if not chunk:
                        break
                    self.serial_mgr.write(chunk)
                    sent += len(chunk)
                    self.log_line(f'[TX] sent {sent}/{size} bytes')
                    time.sleep(0.01)
            self.log_line('[+] File send complete')
        except Exception as e:
            self.log_line(f'[!] File send error: {e}')

    def send_file_fpga_dialog(self):
        path = filedialog.askopenfilename(title='Select file to send to FPGA')
        if not path:
            return
        header = self.parse_header()
        threading.Thread(target=self._send_file_fpga, args=(path, header), daemon=True).start()

    def _send_file_fpga(self, path, header):
        try:
            size = os.path.getsize(path)
            sent = 0
            with open(path, 'rb') as f:
                while True:
                    chunk = f.read(256)
                    if not chunk:
                        break
                    frame = bytes([header]) + chunk
                    self.serial_mgr.write(frame)
                    sent += len(chunk)
                    self.log_line(f'[TX] framed {sent}/{size} bytes')
                    time.sleep(0.005)
            self.log_line('[+] FPGA file send complete')
        except Exception as e:
            self.log_line(f'[!] FPGA send error: {e}')

    def clear_console(self):
        self.text.config(state='normal')
        self.text.delete('1.0', 'end')
        self.text.config(state='disabled')

    def save_log(self):
        path = filedialog.asksaveasfilename(defaultextension='.log', filetypes=[('Log files','*.log'),('All','*.*')])
        if not path:
            return
        with open(path, 'w', encoding='utf8') as f:
            f.write(self.text.get('1.0', 'end'))
        messagebox.showinfo('Saved', f'Log saved to {path}')

    def log_line(self, line):
        self.text.config(state='normal')
        self.text.insert('end', line + '\n')
        self.text.see('end')
        self.text.config(state='disabled')

    def poll_queue(self):
        try:
            while True:
                item = self.serial_mgr.q.get_nowait()
                kind, payload = item
                if kind == 'line':
                    self.log_line(payload)
                elif kind == 'error':
                    self.log_line(f'[ERR] {payload}')
        except queue.Empty:
            pass
        self.after(100, self.poll_queue)

    def on_close(self):
        try:
            self.serial_mgr.close()
        except:
            pass
        self.destroy()

if __name__ == '__main__':
    app = App()
    app.mainloop()
