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

prev=0;t0=time.time()
print('SWA (CH5) Monitor - toggle now!')
print('Time    CH5  Position')
print('-'*35)
data=b''
while time.time()-t0<45:
    data+=os.read(fd,256)
    while len(data)>=25:
        if data[0]!=0x0F or data[24]!=0x00:data=data[1:];continue
        frame=data[:25];data=data[25:]
        i=4;bi=1+(i*11)//8;bo=(i*11)%8
        raw=(frame[bi]|(frame[bi+1]<<8)|(frame[bi+2]<<16))
        ch5=(raw>>bo)&0x07FF
        if ch5!=prev:
            t=time.time()-t0
            if ch5<500: pos='UP'
            elif ch5<800: pos='MID?'
            elif ch5>1500: pos='DOWN'
            else: pos='MID'
            print('%5.1fs  %4d  %s'%(t,ch5,pos))
            prev=ch5
os.close(fd)
print('DONE')
