import socket, threading, time, os, sys, queue
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import numpy as np
import cv2

DEFAULT_IFACE_IP   = "192.168.240.2"
DEFAULT_FPGA_IP    = "192.168.240.1"
PORT_CTRL          = 6003
PORT_IMG_SEND      = 6004
PORT_VIDEO_RX      = 6002
W, H               = 640, 480
LINE_BYTES         = W * 2
HDR_RX_BYTES       = 16
UDP_RX_PAYLOAD     = HDR_RX_BYTES + LINE_BYTES

def now_str(): return time.strftime("%H:%M:%S")

def rgb888_to_rgb565_be_numpy(img_rgb: np.ndarray) -> bytes:
    r = img_rgb[..., 0].astype(np.uint16)
    g = img_rgb[..., 1].astype(np.uint16)
    b = img_rgb[..., 2].astype(np.uint16)
    v = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
    hi = (v >> 8).astype(np.uint8); lo = (v & 0xFF).astype(np.uint8)
    inter = np.empty((img_rgb.shape[0], img_rgb.shape[1]*2), dtype=np.uint8)
    inter[:, 0::2] = hi; inter[:, 1::2] = lo
    return inter.tobytes()

def rgb565_to_bgr888(line_bytes: bytes, big_endian: bool) -> np.ndarray:
    dt = '>u2' if big_endian else '<u2'
    px = np.frombuffer(line_bytes, dtype=dt)
    r5 = (px >> 11) & 0x1F; g6 = (px >> 5) & 0x3F; b5 = px & 0x1F
    r8 = (r5 << 3) | (r5 >> 2); g8 = (g6 << 2) | (g6 >> 4); b8 = (b5 << 3) | (b5 >> 2)
    return np.stack([b8.astype(np.uint8), g8.astype(np.uint8), r8.astype(np.uint8)], axis=1)

def parse_header16(h: bytes):
    if len(h) != 16: return None
    if h[0]!=0x5A or h[1]!=0xA5 or h[2]!=0x01: return None
    flags = h[3]; frame_id = h[4] | (h[5]<<8); line_idx = h[6] | (h[7]<<8)
    data_len = h[10] | (h[11]<<8)
    return bool(flags & 0x02), bool(flags & 0x01), frame_id, line_idx, data_len

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("FPGA UDP 工具箱（8B头）")
        self.geometry("980x700")
        nb = ttk.Notebook(self); nb.pack(fill="both", expand=True)
        self.tab_ctrl = CtrlTab(nb); self.tab_send = ImageSendTab(nb); self.tab_rx=VideoRxTab(nb)
        nb.add(self.tab_ctrl, text="① 控制"); nb.add(self.tab_send, text="② 发送图片(8B)"); nb.add(self.tab_rx, text="③ 接收视频")
        self.status = tk.StringVar(value="Ready")
        ttk.Label(self, textvariable=self.status, anchor="w", relief="sunken").pack(side="bottom", fill="x")
        self.after(500, self._tick)
    def _tick(self):
        self.status.set(f"[{now_str()}] {self.tab_ctrl.status_str.get()} | {self.tab_send.status_str.get()} | {self.tab_rx.status_str.get()}")
        self.after(500, self._tick)

class CtrlTab(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.sock=None; self.status_str=tk.StringVar(value="CTRL: idle")
        self.iface_ip=tk.StringVar(value=DEFAULT_IFACE_IP); self.dst_ip=tk.StringVar(value="192.168.240.255")
        self.port=tk.IntVar(value=PORT_CTRL); self.chk_bcast=tk.BooleanVar(value=True)
        self._ui()
    def _ui(self):
        f=ttk.Frame(self); f.pack(fill="x",pady=8,padx=10)
        ttk.Label(f,text="本机IP").grid(row=0,column=0); ttk.Entry(f,textvariable=self.iface_ip,width=16).grid(row=0,column=1,padx=4)
        ttk.Label(f,text="目标IP").grid(row=0,column=2); ttk.Entry(f,textvariable=self.dst_ip,width=16).grid(row=0,column=3,padx=4)
        ttk.Label(f,text="端口").grid(row=0,column=4); ttk.Entry(f,textvariable=self.port,width=8).grid(row=0,column=5,padx=4)
        ttk.Checkbutton(f,text="全1广播",variable=self.chk_bcast).grid(row=0,column=6,padx=6)
        ttk.Button(f,text="初始化",command=self.ensure_socket).grid(row=0,column=7,padx=6)
        lf=ttk.LabelFrame(self,text="LED"); lf.pack(fill="x",padx=10,pady=6)
        self.led_mask=tk.StringVar(value="0x0F"); self.led_value=tk.StringVar(value="0x0F")
        ttk.Label(lf,text="mask").grid(row=0,column=0); ttk.Entry(lf,textvariable=self.led_mask,width=8).grid(row=0,column=1)
        ttk.Label(lf,text="value").grid(row=0,column=2); ttk.Entry(lf,textvariable=self.led_value,width=8).grid(row=0,column=3)
        ttk.Button(lf,text="发送",command=self.send_led).grid(row=0,column=4,padx=6)
        sf=ttk.LabelFrame(self,text="SEG"); sf.pack(fill="x",padx=10,pady=6)
        self.seg_en=tk.StringVar(value="0x00"); self.seg_pat=tk.StringVar(value="0xF8")
        ttk.Label(sf,text="en").grid(row=0,column=0); ttk.Entry(sf,textvariable=self.seg_en,width=8).grid(row=0,column=1)
        ttk.Label(sf,text="pat").grid(row=0,column=2); ttk.Entry(sf,textvariable=self.seg_pat,width=8).grid(row=0,column=3)
        ttk.Button(sf,text="发送",command=self.send_seg).grid(row=0,column=4,padx=6)
        bf=ttk.LabelFrame(self,text="BOTH"); bf.pack(fill="x",padx=10,pady=6)
        self.b_mask=tk.StringVar(value="0b"); self.b_value=tk.StringVar(value="0b"); self.b_en=tk.StringVar(value="0x"); self.b_pat=tk.StringVar(value="0x")
        for i,(txt,var) in enumerate([("mask",self.b_mask),("value",self.b_value),("en",self.b_en),("pat",self.b_pat)]):
            ttk.Label(bf,text=txt).grid(row=0,column=2*i); ttk.Entry(bf,textvariable=var,width=8).grid(row=0,column=2*i+1)
        ttk.Button(bf,text="发送",command=self.send_both).grid(row=0,column=8,padx=6)
        self.log = tk.Text(self,height=8,font=("Consolas",10)); self.log.pack(fill="both",expand=False,padx=10,pady=8); self.log_insert("就绪。\n")
    def log_insert(self,s): self.log.insert("end",f"[{now_str()}] {s}"); self.log.see("end")
    def ensure_socket(self):
        try:
            if self.sock: self.sock.close()
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)
            self.sock.bind((self.iface_ip.get().strip(),0)); self.status_str.set(f"CTRL: {self.sock.getsockname()[0]}->{self.dst_ip.get()}:{self.port.get()}")
            self.log_insert("初始化完成\n")
        except Exception as e: messagebox.showerror("Socket错误",str(e))
    def _parse_int(self,s):
        s=s.strip().lower()
        if s.startswith('0x'): return int(s,16)
        if s.startswith('0b'): return int(s,2)
        return int(s,10)
    def _send(self,payload:bytes):
        if not self.sock: self.ensure_socket()
        dst=self.dst_ip.get().strip(); port=int(self.port.get())
        try:
            self.sock.sendto(payload,(dst,port))
            if self.chk_bcast.get(): self.sock.sendto(payload,('255.255.255.255',port))
        except Exception as e: self.log_insert(f"发送失败：{e}\n")
    def send_led(self):
        try:
            mask=self._parse_int(self.led_mask.get())&0x0F; val=self._parse_int(self.led_value.get())&0x0F
            self._send(b'LED!'+bytes([mask,val])); self.log_insert(f"LED -> mask=0x{mask:X} value=0x{val:X}\n")
        except Exception as e: self.log_insert(f"解析失败：{e}\n")
    def send_seg(self):
        try:
            en=self._parse_int(self.seg_en.get())&0xFF; pat=self._parse_int(self.seg_pat.get())&0xFF
            self._send(b'SEG!'+bytes([en,pat])); self.log_insert(f"SEG -> en=0x{en:02X} pat=0x{pat:02X}\n")
        except Exception as e: self.log_insert(f"解析失败：{e}\n")
    def send_both(self):
        try:
            m=self._parse_int(self.b_mask.get())&0x0F; v=self._parse_int(self.b_value.get())&0x0F; en=self._parse_int(self.b_en.get())&0xFF; pat=self._parse_int(self.b_pat.get())&0xFF
            self._send(b'BOTH'+bytes([m,v,en,pat])); self.log_insert(f"BOTH -> mask=0x{m:X} value=0x{v:X} en=0x{en:02X} pat=0x{pat:02X}\n")
        except Exception as e: self.log_insert(f"解析失败：{e}\n")

class ImageSendTab(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.sock=None; self.img_path=tk.StringVar(value=""); self.status_str=tk.StringVar(value="SEND: idle")
        self.sleep_per_line=tk.DoubleVar(value=0.0005); self.iface_ip=tk.StringVar(value=DEFAULT_IFACE_IP); self.fpga_ip=tk.StringVar(value=DEFAULT_FPGA_IP); self.port=tk.IntVar(value=PORT_IMG_SEND)
        self.preview_imgtk=None
        self._ui()
    def _ui(self):
        top=ttk.Frame(self); top.pack(fill="x",padx=10,pady=8)
        for i,(txt,var,w) in enumerate([("本机IP",self.iface_ip,16),("板端IP",self.fpga_ip,16),("端口",self.port,8),("每行延时(s)",self.sleep_per_line,8)]):
            ttk.Label(top,text=txt).grid(row=0,column=i*2,sticky="e"); ttk.Entry(top,textvariable=var,width=w).grid(row=0,column=i*2+1,padx=4)
        ttk.Button(top,text="选择图片",command=self.choose_image).grid(row=0,column=8,padx=6); ttk.Button(top,text="发送图片(8B)",command=self.send_image).grid(row=0,column=9,padx=6)
        self.preview=ttk.Label(self,text="预览区（640x480 缩放显示）",relief="sunken"); self.preview.pack(fill="both",expand=True,padx=10,pady=8)
        self.log=tk.Text(self,height=8,font=("Consolas",10)); self.log.pack(fill="x",padx=10,pady=8)
    def log_insert(self,s): self.log.insert("end",f"[{now_str()}] {s}"); self.log.see("end")
    def choose_image(self):
        p=filedialog.askopenfilename(title="选择图片",filetypes=[("Images","*.png;*.jpg;*.jpeg;*.bmp"),("All","*.*")])
        if not p: return
        self.img_path.set(p); self.update_preview(p)
    def update_preview(self,p):
        try:
            img=Image.open(p).convert('RGB').resize((W,H)); imgtk=ImageTk.PhotoImage(img.resize((min(W,640),min(H,480))))
            self.preview_imgtk=imgtk; self.preview.configure(image=imgtk); self.status_str.set(f"SEND: loaded {os.path.basename(p)}")
        except Exception as e: messagebox.showerror("读取失败",str(e))
    def ensure_socket(self):
        try:
            if self.sock: self.sock.close()
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.bind((self.iface_ip.get().strip(),0)); self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4*1024*1024)
            self.status_str.set(f"SEND: {self.sock.getsockname()[0]} -> {self.fpga_ip.get()}:{self.port.get()}")
        except Exception as e: messagebox.showerror("Socket错误",str(e))
    def send_image(self):
        p=self.img_path.get().strip()
        if not p or not os.path.isfile(p): messagebox.showwarning("提示","请先选择图片"); return
        self.ensure_socket(); threading.Thread(target=self._send_worker,args=(p,),daemon=True).start()
    def _send_worker(self,p):
        try:
            img=Image.open(p).convert('RGB').resize((W,H)); img_np=np.array(img,dtype=np.uint8); all_bytes=rgb888_to_rgb565_be_numpy(img_np)
            slp=float(self.sleep_per_line.get()); dst=(self.fpga_ip.get().strip(),int(self.port.get()))
            self.log_insert(f"发送到 {dst[0]}:{dst[1]}，{W}x{H} 共 {H} 行...(8B头)\n")
            for y in range(H):
                off=y*LINE_BYTES; payload=all_bytes[off:off+LINE_BYTES]
                hdr=b'P2FV'+bytes([(y>>8)&0xFF, y&0xFF, (LINE_BYTES>>8)&0xFF, LINE_BYTES&0xFF])
                self.sock.sendto(hdr+payload,dst)
                if slp>0: time.sleep(slp)
            self.log_insert("发送完成。\n")
        except Exception as e: self.log_insert(f"发送失败：{e}\n")

class VideoRxTab(ttk.Frame):
    def __init__(self, master):
        super().__init__(master)
        self.iface_ip=tk.StringVar(value=DEFAULT_IFACE_IP); self.port=tk.IntVar(value=PORT_VIDEO_RX)
        self.big_endian=tk.BooleanVar(value=True); self.running=False; self.sock=None; self.status_str=tk.StringVar(value="RX: idle")
        self.frame_queue=queue.Queue(maxsize=2); self.preview_imgtk=None
        self._ui()
    def _ui(self):
        top=ttk.Frame(self); top.pack(fill="x",padx=10,pady=8)
        ttk.Label(top,text="绑定IP").grid(row=0,column=0,sticky="e"); ttk.Entry(top,textvariable=self.iface_ip,width=16).grid(row=0,column=1,padx=4)
        ttk.Label(top,text="端口").grid(row=0,column=2,sticky="e"); ttk.Entry(top,textvariable=self.port,width=8).grid(row=0,column=3,padx=4)
        ttk.Checkbutton(top,text="像素大端",variable=self.big_endian).grid(row=0,column=4,padx=8)
        self.btn=ttk.Button(top,text="开始接收",command=self.toggle); self.btn.grid(row=0,column=5,padx=6)
        self.preview=ttk.Label(self,text="视频预览",relief="sunken"); self.preview.pack(fill="both",expand=True,padx=10,pady=8)
        self.info=ttk.Label(self,text="FPS: -",anchor="w"); self.info.pack(fill="x",padx=10,pady=(0,8))
    def toggle(self):
        if self.running: self.stop()
        else: self.start()
    def start(self):
        try:
            if self.sock: self.sock.close()
            self.sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM); self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
            self.sock.bind((self.iface_ip.get().strip(),int(self.port.get()))); self.sock.settimeout(1.0)
            self.running=True; self.btn.config(text="停止接收"); self.status_str.set(f"RX: bind {self.iface_ip.get()}:{self.port.get()}")
            threading.Thread(target=self._rx_loop,daemon=True).start(); self.after(10,self._ui_loop)
        except Exception as e: messagebox.showerror("绑定失败",str(e))
    def stop(self):
        self.running=False
        try:
            if self.sock: self.sock.close()
        except: pass
        self.sock=None; self.btn.config(text="开始接收"); self.status_str.set("RX: stopped")
    def _rx_loop(self):
        frame_bgr=np.zeros((H,W,3),np.uint8); cur_frame_id=None; lines_filled=0; t0=time.time(); frames=0
        while self.running:
            try: data,addr=self.sock.recvfrom(2048)
            except socket.timeout: continue
            except Exception: break
            if len(data)!=UDP_RX_PAYLOAD: continue
            hdr=data[:HDR_RX_BYTES]; pay=data[HDR_RX_BYTES:]
            parsed=parse_header16(hdr)
            if parsed is None: continue
            sof,sol,frame_id,line_idx,data_len=parsed
            if data_len!=LINE_BYTES or not (0<=line_idx<H): continue
            if cur_frame_id is None or frame_id!=cur_frame_id: cur_frame_id=frame_id; lines_filled=0
            frame_bgr[line_idx,:,:]=rgb565_to_bgr888(pay,self.big_endian.get()); lines_filled+=1
            if lines_filled>=H:
                frames+=1
                try:
                    if not self.frame_queue.full(): self.frame_queue.put_nowait(frame_bgr.copy())
                except queue.Full: pass
                now=time.time()
                if now-t0>=1.0:
                    self.status_str.set(f"RX: {frames/(now-t0):.1f} FPS, frame_id={cur_frame_id}")
                    frames=0; t0=now
                cur_frame_id+=1; lines_filled=0
    def _ui_loop(self):
        if not self.running: return
        try:
            frame=self.frame_queue.get_nowait()
            img_rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB); img=Image.fromarray(img_rgb)
            w_disp=min(860,W); h_disp=int(H*(w_disp/W)); img=img.resize((w_disp,h_disp),Image.NEAREST)
            imgtk=ImageTk.PhotoImage(img); self.preview_imgtk=imgtk; self.preview.configure(image=imgtk)
        except queue.Empty: pass
        self.after(10,self._ui_loop)

if __name__=="__main__":
    App().mainloop()
