import os,time,struct,fcntl,threading

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

# EXACT vendor readFrame: byte-by-byte, sync 0x0F, collect 25
synced=False;rx=[];prev=[0]*16;t0=time.time()
print("OPERATE REMOTE NOW — 30 seconds")
while time.time()-t0<30:
    try:
        b=os.read(fd,1)
        if not b: time.sleep(0.001); continue
    except: time.sleep(0.001); continue
    if not synced:
        if b[0]==0x0F: synced=True; rx=[b[0]]
        continue
    rx.append(b[0])
    if len(rx)>=25:
        synced=False
        frame=bytes(rx); rx=[]
        ch=[0]*16
        for i in range(16):
            bi=1+(i*11)//8; bo=(i*11)%8
            raw=frame[bi]|(frame[bi+1]<<8)|(frame[bi+2]<<16)
            ch[i]=(raw>>bo)&0x07FF
        changed=False
        for i in range(8):
            if abs(ch[i]-prev[i])>30:
                t=time.time()-t0
                print("%5.1fs CH%d: %4d -> %4d"%(t,i+1,prev[i],ch[i]))
                prev[i]=ch[i]; changed=True
os.close(fd)
print("DONE")
