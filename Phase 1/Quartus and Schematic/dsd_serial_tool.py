import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading, time, os, queue
from datetime import datetime
import serial, serial.tools.list_ports

DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.5

class SerialManager:
    def __init__(self, port=None, baud=DEFAULT_BAUD, timeout=READ_TIMEOUT, log_path=None):
        self.port = port
        self.baud = int(baud)
        self.timeout = timeout
        self.ser = None
        self.reader_thread = None
        self._stop = threading.Event()
        self.log_path = log_path
        self.log_file = None
        if log_path:
            try:
                self.log_file = open(log_path, "a", buffering=1, encoding="utf8")
            except IOError:
                self.log_file = None
        self.q = queue.Queue()

    def open(self):
        if not self.port:
            raise RuntimeError('port not set')
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.1)
        except Exception as e:
            raise
        self._stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def close(self):
        self._stop.set()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.log_file:
            try:
                self.log_file.close()
            except:
                pass

    def start_reader(self):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('serial port not open')
        if self.reader_thread and self.reader_thread.is_alive():
            return
        self._stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                data = self.ser.readline()
            except Exception as e:
                if not self._stop.is_set():
                    self.q.put(('error', str(e)))
                break
            if not data:
                continue
            ts = datetime.utcnow().isoformat(sep=' ', timespec='milliseconds')
            try:
                text = data.decode('utf-8', errors='replace').rstrip('')
            except:
                text = repr(data)
            out = f"{ts} {text}"
            self.q.put(('line', out))
            if self.log_file:
                try:
                    self.log_file.write(out + "")
                except:
                    pass

    def write(self, data: bytes):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('serial port not open')
        try:
            self.ser.write(data)
        except Exception as e:
            raise

    def toggle_reset(self, rst_ms=100, use_dtr=True):
        if not self.ser:
            raise RuntimeError('serial port not open')
        try:
            if use_dtr:
                self.ser.dtr = False
                time.sleep(rst_ms / 1000.0)
                self.ser.dtr = True
            else:
                self.ser.rts = False
                time.sleep(rst_ms / 1000.0)
                self.ser.rts = True
            time.sleep(0.05)
            self.q.put(('line','[RESET] toggled'))
        except Exception as e:
            self.q.put(('error', str(e)))

def list_serial_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('DSD-i1 Serial Console')
        self.geometry('1000x640')
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        self.serial_mgr = SerialManager()
        self.create_widgets()
        self.after(100, self.poll_queue)

    def create_widgets(self):
        top = ttk.Frame(self)
        top.pack(side='top', fill='x', padx=6, pady=6)

        ttk.Label(top, text='Port').pack(side='left')
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, values=list_serial_ports(), width=14)
        self.port_combo.pack(side='left', padx=4)

        ttk.Button(top, text='Refresh', command=self.refresh_ports).pack(side='left', padx=4)

        ttk.Label(top, text='Baud').pack(side='left', padx=(8,0))
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        ttk.Entry(top, textvariable=self.baud_var, width=8).pack(side='left', padx=4)

        self.connect_btn = ttk.Button(top, text='Connect', command=self.toggle_connect)
        self.connect_btn.pack(side='left', padx=6)

        self.status_lbl = ttk.Label(top, text='Disconnected', foreground='red')
        self.status_lbl.pack(side='left', padx=8)

        self.reset_btn = ttk.Button(top, text='Reset', command=self.reset_board)
        self.reset_btn.pack(side='left', padx=4)

        ttk.Label(top, text='Mode').pack(side='left', padx=(12,0))
        self.mode_var = tk.StringVar(value='MCU')
        ttk.OptionMenu(top, self.mode_var, 'MCU', 'MCU', 'FPGA').pack(side='left', padx=4)

        ttk.Label(top, text='Header').pack(side='left', padx=(8,0))
        self.header_var = tk.StringVar(value='0xA5')
        ttk.Entry(top, textvariable=self.header_var, width=6).pack(side='left', padx=4)

        ttk.Button(top, text='Send File', command=self.send_file_plain_dialog).pack(side='left', padx=6)
        ttk.Button(top, text='Send File (FPGA)', command=self.send_file_fpga_dialog).pack(side='left', padx=4)

        mid = ttk.Frame(self)
        mid.pack(side='top', fill='both', expand=True, padx=6, pady=6)

        left = ttk.Frame(mid)
        left.pack(side='left', fill='both', expand=True)

        self.text = tk.Text(left, wrap='none')
        self.text.pack(side='left', fill='both', expand=True)
        self.text.config(state='disabled')

        vsb = ttk.Scrollbar(left, orient='vertical', command=self.text.yview)
        vsb.pack(side='right', fill='y')
        self.text['yscrollcommand'] = vsb.set

        right = ttk.Frame(mid, width=260)
        right.pack(side='right', fill='y')

        ttk.Label(right, text='Controls').pack(anchor='nw', padx=6, pady=(6,2))
        ttk.Button(right, text='Clear', command=self.clear_console).pack(fill='x', padx=6, pady=2)
        ttk.Button(right, text='Save Log', command=self.save_log).pack(fill='x', padx=6, pady=2)

        ttk.Separator(right, orient='horizontal').pack(fill='x', pady=6, padx=6)

        ttk.Label(right, text='Quick Commands').pack(anchor='nw', padx=6)
        self.quick_frame = ttk.Frame(right)
        self.quick_frame.pack(fill='x', padx=6, pady=4)
        ttk.Button(self.quick_frame, text='Ping', command=lambda: self.quick_send('ping')).pack(fill='x', pady=2)
        ttk.Button(self.quick_frame, text='Status', command=lambda: self.quick_send('status')).pack(fill='x', pady=2)

        bottom = ttk.Frame(self)
        bottom.pack(side='bottom', fill='x', padx=6, pady=6)

        self.entry_var = tk.StringVar()
        self.entry = ttk.Entry(bottom, textvariable=self.entry_var)
        self.entry.pack(side='left', fill='x', expand=True, padx=(0,6))
        self.entry.bind('<Return>', self.on_send)

        ttk.Button(bottom, text='Send', command=self.on_send).pack(side='left')

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo['values'] = ports

    def toggle_connect(self):
        if not self.serial_mgr.ser:
            port = self.port_var.get() or None
            if not port:
                messagebox.showerror('Error', 'Select a serial port')
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
            self.status_lbl.config(text=f'Connected {port} @ {baud}', foreground='green')
            self.serial_mgr.q.put(('line', f'[+] Connected {port} @ {baud}'))
        else:
            try:
                self.serial_mgr.close()
            except:
                pass
            self.connect_btn.config(text='Connect')
            self.status_lbl.config(text='Disconnected', foreground='red')
            self.serial_mgr = SerialManager()

    def reset_board(self):
        try:
            self.serial_mgr.toggle_reset()
        except Exception as e:
            messagebox.showerror('Error', f'Reset failed: {e}')

    def on_send(self, event=None):
        text = self.entry_var.get()
        if not text:
            return
        tosend = text + ''
        try:
            if self.mode_var.get() == 'FPGA':
                try:
                    header = int(self.header_var.get(), 0) & 0xFF
                except:
                    header = 0xA5
                frame = bytes([header]) + tosend.encode('utf-8', errors='replace')
                self.serial_mgr.write(frame)
            else:
                self.serial_mgr.write(tosend.encode('utf-8', errors='replace'))
            self.log_line(f'[TX] {text}')
        except Exception as e:
            messagebox.showerror('Error', f'Send failed: {e}')
        self.entry_var.set('')

    def quick_send(self, cmd):
        self.entry_var.set(cmd)
        self.on_send()

    def send_file_plain_dialog(self):
        path = filedialog.askopenfilename(title='Select file to send')
        if not path:
            return
        threading.Thread(target=self._send_file_plain, args=(path,), daemon=True).start()

    def _send_file_plain(self, path):
        if not os.path.exists(path):
            self.log_line(f'[!] File not found: {path}')
            return
        size = os.path.getsize(path)
        sent = 0
        try:
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
        try:
            header = int(self.header_var.get(), 0) & 0xFF
        except:
            header = 0xA5
        threading.Thread(target=self._send_file_fpga, args=(path, header), daemon=True).start()

    def _send_file_fpga(self, path, header):
        if not os.path.exists(path):
            self.log_line(f'[!] File not found: {path}')
            return
        size = os.path.getsize(path)
        sent = 0
        try:
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
        self.text.insert('end', line + '')
        self.text.see('end')
        self.text.config(state='disabled')

    def poll_queue(self):
        try:
            while True:
                kind, payload = self.serial_mgr.q.get_nowait()
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
