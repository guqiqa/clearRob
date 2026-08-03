import os,time,struct,fcntl
fd=os.open('/dev/ttyS1',os.O_RDWR|os.O_NOCTTY|os.O_NDELAY)
TCGETS2,TCSETS2=0x802c542a,0x402c542b
buf=bytearray(72);fcntl.ioctl(fd,TCGETS2,buf)
cflag=struct.unpack_from('I',buf,8)[0]
cflag&=0xffffffcf;cflag|=0x30|0x100|0x40|0x880;cflag&=~0x200
struct.pack_into('I',buf,8,cflag);buf[22]=0;buf[23]=0
cflag2=struct.unpack_from('I',buf,8)[0];cflag2&=0xffffeff0;cflag2|=0x1000
struct.pack_into('I',buf,8,cflag2)
for off in(36,40,40,44):struct.pack_into('I',buf,off,100000)
fcntl.ioctl(fd,TCSETS2,buf)
time.sleep(0.3)

SBUS_LEN=25;prev=[0]*16;t0=time.time();data=b''
print('READY - toggle SWA low/mid/high now!')
while time.time()-t0<20:
    data+=os.read(fd,256)
    while len(data)>=SBUS_LEN:
        hdr=data.find(b'\x0f')
        if hdr<0:data=b'';break
        if hdr>0:data=data[hdr:]
        if len(data)<SBUS_LEN:break
        if data[SBUS_LEN-1]!=0x00:data=data[1:];continue
        frame=data[:SBUS_LEN];data=data[SBUS_LEN:]
        chs=[0]*16;bp=bi=0
        for ch in range(16):
            v=bits=0
            while bits<11:
                if bp>=8:bi+=1;bp=0
                n=min(11-bits,8-bp)
                v|=((frame[bi]>>bp)&((1<<n)-1))<<bits
                bp+=n;bits+=n
            chs[ch]=v
        changed=False
        for i in range(8):
            if abs(chs[i]-prev[i])>30:
                changed=True
                t=time.time()-t0
                print("%.1fs: CH%d %d->%d"%(t,i+1,prev[i],chs[i]))
                prev[i]=chs[i]
os.close(fd)
