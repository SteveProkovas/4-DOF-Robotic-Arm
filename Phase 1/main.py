import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import time
import os
import queue
from datetime import datetime
import serial
import serial.tools.list_ports

DEFAULT_BAUD = 115200
READ_TIMEOUT = 0.5
RECONNECT_DELAY = 2.0  # Delay before attempting reconnect

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
        self.auto_reconnect = False
        self.connection_callback = None
        if log_path:
            self.open_log_file(log_path)
        self.q = queue.Queue()

    def open_log_file(self, log_path):
        """Open log file with error handling"""
        try:
            self.log_file = open(log_path, "a", buffering=1, encoding="utf-8")
            self.log_path = log_path
        except IOError as e:
            self.q.put(('error', f'Failed to open log file: {e}'))
            self.log_file = None

    def set_connection_callback(self, callback):
        """Set callback for connection status changes"""
        self.connection_callback = callback

    def open(self):
        if not self.port:
            raise RuntimeError('Port not set')
        
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            time.sleep(0.1)
            if self.connection_callback:
                self.connection_callback(True, None)
        except Exception as e:
            if self.connection_callback:
                self.connection_callback(False, str(e))
            raise
        
        self._stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def close(self, notify=True):
        self._stop.set()
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                self.q.put(('error', f'Error closing port: {e}'))
        
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        
        if notify and self.connection_callback:
            self.connection_callback(False, "Manually disconnected")
            
    def start_reader(self):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        if self.reader_thread and self.reader_thread.is_alive():
            return
        self._stop.clear()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def _reader(self):
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    if self.auto_reconnect:
                        time.sleep(RECONNECT_DELAY)
                        try:
                            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                            self.q.put(('info', f'Reconnected to {self.port}'))
                            if self.connection_callback:
                                self.connection_callback(True, None)
                        except Exception as e:
                            self.q.put(('error', f'Reconnection failed: {e}'))
                            continue
                    else:
                        break
                
                data = self.ser.readline()
            except Exception as e:
                if not self._stop.is_set():
                    self.q.put(('error', f'Read error: {str(e)}'))
                    if self.connection_callback:
                        self.connection_callback(False, str(e))
                break
                
            if not data:
                continue
                
            ts = datetime.utcnow().isoformat(sep=' ', timespec='milliseconds')
            try:
                text = data.decode('utf-8', errors='replace').rstrip()
            except UnicodeDecodeError:
                # Try to decode as hexadecimal representation for binary data
                text = ' '.join(f'{b:02X}' for b in data)
                
            out = f"{ts} {text}"
            self.q.put(('line', out))
            
            if self.log_file:
                try:
                    self.log_file.write(out + "\n")
                except Exception as e:
                    self.q.put(('error', f'Log write error: {e}'))
                    self.log_file = None

    def write(self, data: bytes):
        if not self.ser or not self.ser.is_open:
            raise RuntimeError('Serial port not open')
        try:
            self.ser.write(data)
        except Exception as e:
            if self.connection_callback:
                self.connection_callback(False, str(e))
            raise

    def toggle_reset(self, rst_ms=100, use_dtr=True):
        if not self.ser:
            raise RuntimeError('Serial port not open')
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
            self.q.put(('info', '[RESET] toggled'))
        except Exception as e:
            self.q.put(('error', str(e)))


def list_serial_ports():
    return sorted([p.device for p in serial.tools.list_ports.comports()])


class ScrollableTextFrame(ttk.Frame):
    """A frame containing a text widget and scrollbar"""
    def __init__(self, parent, **kwargs):
        super().__init__(parent, **kwargs)
        self.text = tk.Text(self, wrap='word', undo=True)
        self.v_scrollbar = ttk.Scrollbar(self, orient='vertical', command=self.text.yview)
        self.text.configure(yscrollcommand=self.v_scrollbar.set)
        
        self.text.grid(row=0, column=0, sticky='nsew')
        self.v_scrollbar.grid(row=0, column=1, sticky='ns')
        
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)
        
    def write(self, text):
        self.text.insert('end', text)
        self.text.see('end')


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('DSD-i1 Serial Console')
        self.geometry('1000x640')
        self.minsize(800, 500)
        self.protocol('WM_DELETE_WINDOW', self.on_close)
        
        # Initialize serial manager
        self.serial_mgr = SerialManager()
        self.serial_mgr.set_connection_callback(self.on_connection_change)
        
        # Settings
        self.auto_scroll = tk.BooleanVar(value=True)
        self.show_timestamps = tk.BooleanVar(value=True)
        self.auto_reconnect = tk.BooleanVar(value=False)
        
        self.create_widgets()
        self.after(100, self.poll_queue)
        
        # Bind Ctrl+C and Ctrl+V for text widget
        self.text_widget.bind('<Control-c>', self.copy_text)
        self.text_widget.bind('<Control-v>', self.paste_text)

    def create_widgets(self):
        # Create paned window for resizable panels
        main_paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)
        
        # Left panel (console)
        left_frame = ttk.Frame(main_paned)
        main_paned.add(left_frame, weight=3)
        
        # Right panel (controls)
        right_frame = ttk.Frame(main_paned, width=260)
        main_paned.add(right_frame, weight=1)
        
        # Build console area
        self.build_console(left_frame)
        
        # Build control panel
        self.build_control_panel(right_frame)
        
        # Build bottom panel (entry and send button)
        self.build_bottom_panel()

    def build_console(self, parent):
        # Console frame
        console_frame = ttk.LabelFrame(parent, text="Console Output")
        console_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 6))
        
        # Text widget with scrollbar
        self.text_widget = tk.Text(console_frame, wrap='word', state='disabled')
        text_scrollbar = ttk.Scrollbar(console_frame, orient='vertical', command=self.text_widget.yview)
        self.text_widget.configure(yscrollcommand=text_scrollbar.set)
        
        self.text_widget.pack(side='left', fill='both', expand=True)
        text_scrollbar.pack(side='right', fill='y')
        
        # Console controls
        console_controls = ttk.Frame(console_frame)
        console_controls.pack(side='bottom', fill='x', pady=2)
        
        ttk.Checkbutton(console_controls, text="Auto Scroll", variable=self.auto_scroll).pack(side='left', padx=4)
        ttk.Checkbutton(console_controls, text="Timestamps", variable=self.show_timestamps).pack(side='left', padx=4)
        ttk.Button(console_controls, text="Clear", command=self.clear_console).pack(side='right', padx=4)

    def build_control_panel(self, parent):
        # Connection settings
        conn_frame = ttk.LabelFrame(parent, text="Connection Settings")
        conn_frame.pack(fill='x', pady=(0, 6))
        
        ttk.Label(conn_frame, text='Port:').grid(row=0, column=0, sticky='w', padx=4, pady=2)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=15)
        self.port_combo.grid(row=0, column=1, sticky='we', padx=4, pady=2)
        ttk.Button(conn_frame, text='Refresh', command=self.refresh_ports).grid(row=0, column=2, padx=4, pady=2)
        
        ttk.Label(conn_frame, text='Baud:').grid(row=1, column=0, sticky='w', padx=4, pady=2)
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, width=15)
        baud_combo['values'] = ('9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600')
        baud_combo.grid(row=1, column=1, sticky='we', padx=4, pady=2)
        
        ttk.Checkbutton(conn_frame, text="Auto Reconnect", variable=self.auto_reconnect, 
                       command=self.toggle_auto_reconnect).grid(row=2, column=0, columnspan=2, sticky='w', padx=4, pady=2)
        
        self.connect_btn = ttk.Button(conn_frame, text='Connect', command=self.toggle_connect)
        self.connect_btn.grid(row=3, column=0, columnspan=2, sticky='we', padx=4, pady=2)
        
        self.status_lbl = ttk.Label(conn_frame, text='Disconnected', foreground='red')
        self.status_lbl.grid(row=4, column=0, columnspan=3, sticky='w', padx=4, pady=2)
        
        conn_frame.columnconfigure(1, weight=1)
        
        # Device control
        ctrl_frame = ttk.LabelFrame(parent, text="Device Control")
        ctrl_frame.pack(fill='x', pady=(0, 6))
        
        ttk.Label(ctrl_frame, text='Mode:').grid(row=0, column=0, sticky='w', padx=4, pady=2)
        self.mode_var = tk.StringVar(value='MCU')
        mode_combo = ttk.Combobox(ctrl_frame, textvariable=self.mode_var, state='readonly', width=8)
        mode_combo['values'] = ('MCU', 'FPGA')
        mode_combo.grid(row=0, column=1, sticky='w', padx=4, pady=2)
        
        ttk.Label(ctrl_frame, text='Header:').grid(row=1, column=0, sticky='w', padx=4, pady=2)
        self.header_var = tk.StringVar(value='0xA5')
        ttk.Entry(ctrl_frame, textvariable=self.header_var, width=8).grid(row=1, column=1, sticky='w', padx=4, pady=2)
        
        self.reset_btn = ttk.Button(ctrl_frame, text='Reset Device', command=self.reset_board)
        self.reset_btn.grid(row=2, column=0, columnspan=2, sticky='we', padx=4, pady=2)
        
        # File transfer
        file_frame = ttk.LabelFrame(parent, text="File Transfer")
        file_frame.pack(fill='x', pady=(0, 6))
        
        ttk.Button(file_frame, text='Send File (Plain)', command=self.send_file_plain_dialog).pack(fill='x', padx=4, pady=2)
        ttk.Button(file_frame, text='Send File (FPGA)', command=self.send_file_fpga_dialog).pack(fill='x', padx=4, pady=2)
        ttk.Button(file_frame, text='Save Log', command=self.save_log).pack(fill='x', padx=4, pady=2)
        
        # Quick commands
        cmd_frame = ttk.LabelFrame(parent, text="Quick Commands")
        cmd_frame.pack(fill='x')
        
        ttk.Button(cmd_frame, text='Ping', command=lambda: self.quick_send('ping')).pack(fill='x', padx=4, pady=2)
        ttk.Button(cmd_frame, text='Status', command=lambda: self.quick_send('status')).pack(fill='x', padx=4, pady=2)
        ttk.Button(cmd_frame, text='Version', command=lambda: self.quick_send('version')).pack(fill='x', padx=4, pady=2)

    def build_bottom_panel(self):
        # Bottom panel for sending commands
        bottom = ttk.Frame(self)
        bottom.pack(side='bottom', fill='x', padx=6, pady=6)
        
        self.entry_var = tk.StringVar()
        self.entry = ttk.Entry(bottom, textvariable=self.entry_var)
        self.entry.pack(side='left', fill='x', expand=True, padx=(0, 6))
        self.entry.bind('<Return>', self.on_send)
        self.entry.bind('<Up>', self.history_previous)
        self.entry.bind('<Down>', self.history_next)
        
        ttk.Button(bottom, text='Send', command=self.on_send).pack(side='left')
        
        # Command history
        self.command_history = []
        self.history_index = -1

    def refresh_ports(self):
        ports = list_serial_ports()
        self.port_combo['values'] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_auto_reconnect(self):
        self.serial_mgr.auto_reconnect = self.auto_reconnect.get()

    def on_connection_change(self, connected, message):
        """Callback for connection status changes"""
        if connected:
            self.connect_btn.config(text='Disconnect')
            self.status_lbl.config(text=f'Connected {self.serial_mgr.port} @ {self.serial_mgr.baud}', foreground='green')
            self.log_line(f'[INFO] Connected {self.serial_mgr.port} @ {self.serial_mgr.baud}')
        else:
            self.connect_btn.config(text='Connect')
            self.status_lbl.config(text=f'Disconnected: {message}', foreground='red')
            if message:
                self.log_line(f'[ERROR] {message}')

    def toggle_connect(self):
        if not self.serial_mgr.ser or not self.serial_mgr.ser.is_open:
            # Connect
            port = self.port_var.get() or None
            if not port:
                messagebox.showerror('Error', 'Select a serial port')
                return
            try:
                baud = int(self.baud_var.get())
            except ValueError:
                messagebox.showerror('Error', 'Invalid baud rate')
                return
                
            self.serial_mgr.port = port
            self.serial_mgr.baud = baud
            self.serial_mgr.auto_reconnect = self.auto_reconnect.get()
            
            try:
                self.serial_mgr.open()
            except Exception as e:
                messagebox.showerror('Error', f'Could not open port: {e}')
        else:
            # Disconnect
            try:
                self.serial_mgr.close()
            except Exception as e:
                messagebox.showerror('Error', f'Error disconnecting: {e}')

    def reset_board(self):
        if not self.serial_mgr.ser or not self.serial_mgr.ser.is_open:
            messagebox.showerror('Error', 'Not connected to any device')
            return
        try:
            self.serial_mgr.toggle_reset()
        except Exception as e:
            messagebox.showerror('Error', f'Reset failed: {e}')

    def on_send(self, event=None):
        text = self.entry_var.get().strip()
        if not text:
            return
            
        # Add to command history
        if not self.command_history or self.command_history[-1] != text:
            self.command_history.append(text)
        self.history_index = len(self.command_history)
        
        try:
            if self.mode_var.get() == 'FPGA':
                try:
                    header = int(self.header_var.get(), 0) & 0xFF
                except ValueError:
                    header = 0xA5
                frame = bytes([header]) + text.encode('utf-8', errors='replace')
                self.serial_mgr.write(frame)
            else:
                self.serial_mgr.write(text.encode('utf-8', errors='replace'))
            self.log_line(f'[TX] {text}')
        except Exception as e:
            messagebox.showerror('Error', f'Send failed: {e}')
        self.entry_var.set('')

    def history_previous(self, event):
        if self.command_history and self.history_index > 0:
            self.history_index -= 1
            self.entry_var.set(self.command_history[self.history_index])
        return "break"

    def history_next(self, event):
        if self.command_history and self.history_index < len(self.command_history) - 1:
            self.history_index += 1
            self.entry_var.set(self.command_history[self.history_index])
        else:
            self.history_index = len(self.command_history)
            self.entry_var.set('')
        return "break"

    def quick_send(self, cmd):
        self.entry_var.set(cmd)
        self.on_send()

    def send_file_plain_dialog(self):
        if not self.serial_mgr.ser or not self.serial_mgr.ser.is_open:
            messagebox.showerror('Error', 'Not connected to any device')
            return
            
        path = filedialog.askopenfilename(title='Select file to send')
        if not path:
            return
        threading.Thread(target=self._send_file_plain, args=(path,), daemon=True).start()

    def _send_file_plain(self, path):
        if not os.path.exists(path):
            self.log_line(f'[ERROR] File not found: {path}')
            return
            
        size = os.path.getsize(path)
        sent = 0
        chunk_size = 1024
        
        try:
            with open(path, 'rb') as f:
                while not self.serial_mgr._stop.is_set():
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    self.serial_mgr.write(chunk)
                    sent += len(chunk)
                    self.log_line(f'[INFO] Sent {sent}/{size} bytes ({sent/size*100:.1f}%)')
                    time.sleep(0.01)
            self.log_line('[INFO] File send complete')
        except Exception as e:
            self.log_line(f'[ERROR] File send error: {e}')

    def send_file_fpga_dialog(self):
        if not self.serial_mgr.ser or not self.serial_mgr.ser.is_open:
            messagebox.showerror('Error', 'Not connected to any device')
            return
            
        path = filedialog.askopenfilename(title='Select file to send to FPGA')
        if not path:
            return
            
        try:
            header = int(self.header_var.get(), 0) & 0xFF
        except ValueError:
            header = 0xA5
            
        threading.Thread(target=self._send_file_fpga, args=(path, header), daemon=True).start()

    def _send_file_fpga(self, path, header):
        if not os.path.exists(path):
            self.log_line(f'[ERROR] File not found: {path}')
            return
            
        size = os.path.getsize(path)
        sent = 0
        chunk_size = 256  # Smaller chunks for FPGA
        
        try:
            with open(path, 'rb') as f:
                while not self.serial_mgr._stop.is_set():
                    chunk = f.read(chunk_size)
                    if not chunk:
                        break
                    frame = bytes([header]) + chunk
                    self.serial_mgr.write(frame)
                    sent += len(chunk)
                    self.log_line(f'[INFO] Sent framed {sent}/{size} bytes ({sent/size*100:.1f}%)')
                    time.sleep(0.005)
            self.log_line('[INFO] FPGA file send complete')
        except Exception as e:
            self.log_line(f'[ERROR] FPGA send error: {e}')

    def clear_console(self):
        self.text_widget.config(state='normal')
        self.text_widget.delete('1.0', 'end')
        self.text_widget.config(state='disabled')

    def save_log(self):
        path = filedialog.asksaveasfilename(
            defaultextension='.log',
            filetypes=[('Log files', '*.log'), ('Text files', '*.txt'), ('All files', '*.*')]
        )
        if not path:
            return
            
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(self.text_widget.get('1.0', 'end-1c'))
            messagebox.showinfo('Saved', f'Log saved to {path}')
        except Exception as e:
            messagebox.showerror('Error', f'Failed to save log: {e}')

    def log_line(self, line):
        self.text_widget.config(state='normal')
        self.text_widget.insert('end', line + '\n')
        if self.auto_scroll.get():
            self.text_widget.see('end')
        self.text_widget.config(state='disabled')

    def poll_queue(self):
        try:
            while True:
                kind, payload = self.serial_mgr.q.get_nowait()
                if kind == 'line':
                    self.log_line(payload)
                elif kind == 'error':
                    self.log_line(f'[ERROR] {payload}')
                elif kind == 'info':
                    self.log_line(f'[INFO] {payload}')
        except queue.Empty:
            pass
        self.after(100, self.poll_queue)

    def copy_text(self, event=None):
        try:
            selected = self.text_widget.get('sel.first', 'sel.last')
            self.clipboard_clear()
            self.clipboard_append(selected)
        except tk.TclError:
            # No text selected
            pass
        return "break"

    def paste_text(self, event=None):
        try:
            clipboard = self.clipboard_get()
            self.text_widget.insert('insert', clipboard)
        except tk.TclError:
            # Clipboard is empty or contains non-text data
            pass
        return "break"

    def on_close(self):
        try:
            self.serial_mgr.close(notify=False)
        except Exception:
            pass
        self.destroy()


if __name__ == '__main__':
    app = App()
    app.mainloop()
