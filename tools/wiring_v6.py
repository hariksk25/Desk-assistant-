from PIL import Image, ImageDraw, ImageFont
W,H=1800,1420
img=Image.new("RGB",(W,H),"white"); d=ImageDraw.Draw(img)
F=lambda s:ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",s)
FB=lambda s:ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",s)
C={"5V":"#d62728","3V3":"#ff7f0e","GND":"#222222","SCK":"#1f77b4","MOSI":"#17becf","DC":"#9467bd","RST":"#8c564b","CS":"#e377c2","SDA":"#2ca02c","SCL":"#006400","ENC":"#7f7f7f","DATA":"#bcbd22","PZ":"#555555"}
d.text((30,20),"Desk Plane Radar v6 wiring (ESP32-S3-DevKitC-1 N16R8)",font=FB(28),fill="black")
d.text((30,58),"Power enters only through the S3's UART USB-C port. Rings and buzzer take 5 V from the 5V pin, everything else 3V3. All GNDs common.",font=F(16),fill="#444")

# board
bx0,by0,bx1,by1=760,140,1040,1130
d.rounded_rectangle((bx0,by0,bx1,by1),12,fill="#e8f0e8",outline="black",width=3)
d.text((bx0+40,by0+12),"ESP32-S3-DevKitC-1",font=FB(18),fill="black")
d.rectangle((bx0+90,by0-2,bx0+150,by0+14),fill="#999"); d.text((bx0+160,by0+40),"USB-C (UART) in",font=F(13),fill="#444")
pinsL=[("3V3","3V3"),("GND","GND"),("12  SCK","SCK"),("11  MOSI","MOSI"),("13  DC","DC"),("14  RST","RST"),("10  CS1","CS"),("16  CS2","CS"),("8   SDA","SDA"),("9   SCL","SCL")]
pinsR=[("5V","5V"),("GND","GND"),("4   ENC1 CLK","ENC"),("5   ENC1 DT","ENC"),("6   ENC1 SW","ENC"),("15  ENC2 CLK","ENC"),("17  ENC2 DT","ENC"),("18  ENC2 SW","ENC"),("47  RING1 DIN","DATA"),("21  RING2 DIN","DATA"),("7   PIEZO","PZ")]
PL={}; PR={}
for i,(lab,net) in enumerate(pinsL):
    y=by0+90+i*90; PL[lab]=(bx0,y)
    d.ellipse((bx0-7,y-7,bx0+7,y+7),fill=C[net]); d.text((bx0+14,y-9),lab,font=FB(15),fill="black")
for i,(lab,net) in enumerate(pinsR):
    y=by0+90+i*82; PR[lab]=(bx1,y)
    d.ellipse((bx1-7,y-7,bx1+7,y+7),fill=C[net]); 
    tw=d.textlength(lab,font=FB(15)); d.text((bx1-16-tw,y-9),lab,font=FB(15),fill="black")

def module(x,y,w,h,title,pins,side):
    d.rounded_rectangle((x,y,x+w,y+h),10,fill="#f4f4f4",outline="black",width=2)
    d.text((x+10,y+8),title,font=FB(16),fill="black")
    out={}
    for i,(lab,net) in enumerate(pins):
        py=y+40+i*24
        px=x+w if side=="L" else x
        d.ellipse((px-5,py-5,px+5,py+5),fill=C[net])
        if side=="L":
            tw=d.textlength(lab,font=F(14)); d.text((px-12-tw,py-8),lab,font=F(14),fill="black")
        else: d.text((px+12,py-8),lab,font=F(14),fill="black")
        out[lab]=(px,py)
    return out

# left modules
tft=[("VCC","3V3"),("GND","GND"),("SCL","SCK"),("SDA","MOSI"),("RES","RST"),("DC","DC"),("CS","CS")]
oled=[("GND","GND"),("VCC","3V3"),("SCL","SCL"),("SDA","SDA")]
T1=module(60,120,300,220,"Round TFT 1 (right dial, GC9A01)",tft,"L")
T2=module(60,380,300,220,"Round TFT 2 (left dial, GC9A01)",tft,"L")
O1=module(60,640,300,150,"OLED 1, 0x3C (upper)",oled,"L")
O2=module(60,830,300,150,"OLED 2, 0x3D (lower), see note",oled,"L")
# right modules
enc=[("+","3V3"),("GND","GND"),("CLK (SIA)","ENC"),("DT (SIB)","ENC"),("SW","ENC")]
ring=[("5V","5V"),("GND","GND"),("DIN","DATA")]
E1=module(1440,120,320,170,"Encoder 1 (right knob, KY-040)",enc,"R")
E2=module(1440,330,320,170,"Encoder 2 (left knob, SmartElex)",enc,"R")
R1=module(1440,560,320,130,"LED ring 1 (right dial, 24 x WS2812B)",ring,"R")
R2=module(1440,730,320,130,"LED ring 2 (left dial, 24 x WS2812B)",ring,"R")
PZ=module(1440,900,320,100,"Passive buzzer",[("+","PZ"),("-","GND")],"R")

def wire(a,b,net,xm,w=3):
    d.line([a,(xm,a[1]),(xm,b[1]),b],fill=C[net],width=w)

# left buses: 3V3 at x=430, GND at x=460 (vertical), then to board pins
d.line([(430,T1["VCC"][1]),(430,O2["VCC"][1])],fill=C["3V3"],width=4); d.line([(430,PL["3V3"][1]),PL["3V3"]],fill=C["3V3"],width=4)
d.line([(465,T1["GND"][1]),(465,O2["GND"][1])],fill=C["GND"],width=4); d.line([(465,PL["GND"][1]),PL["GND"]],fill=C["GND"],width=4)
for M in (T1,T2,O1,O2):
    d.line([M["VCC"],(430,M["VCC"][1])],fill=C["3V3"],width=3)
    d.line([M["GND"],(465,M["GND"][1])],fill=C["GND"],width=3)
# shared SPI buses
for net,lab,bus in (("SCK","SCL",500),("MOSI","SDA",530),("RST","RES",560),("DC","DC",590)):
    pin={"SCK":"12  SCK","MOSI":"11  MOSI","RST":"14  RST","DC":"13  DC"}[net]
    ys=[T1[lab][1],T2[lab][1],PL[pin][1]]
    d.line([(bus,min(ys)),(bus,max(ys))],fill=C[net],width=3)
    for M in (T1,T2): d.line([M[lab],(bus,M[lab][1])],fill=C[net],width=3)
    d.line([(bus,PL[pin][1]),PL[pin]],fill=C[net],width=3)
wire(T1["CS"],PL["10  CS1"],"CS",640); wire(T2["CS"],PL["16  CS2"],"CS",660)
for net,lab,bus,pin in (("SDA","SDA",620,"8   SDA"),("SCL","SCL",690,"9   SCL")):
    ys=[O1[lab][1],O2[lab][1],PL[pin][1]]
    d.line([(bus,min(ys)),(bus,max(ys))],fill=C[net],width=3)
    for M in (O1,O2): d.line([M[lab],(bus,M[lab][1])],fill=C[net],width=3)
    d.line([(bus,PL[pin][1]),PL[pin]],fill=C[net],width=3)

# right side: 3V3 for encoders comes from the left 3V3 bus around the top
d.line([(430,T1["VCC"][1]),(430,100),(1400,100),(1400,E2["+"][1])],fill=C["3V3"],width=4)
for M in (E1,E2): d.line([(1400,M["+"][1]),M["+"]],fill=C["3V3"],width=3)
# GND bus right at 1365
d.line([PR["GND"],(1365,PR["GND"][1])],fill=C["GND"],width=4); d.line([(1365,E1["GND"][1]),(1365,PZ["-"][1])],fill=C["GND"],width=4)
for M in (E1,E2,R1,R2,PZ):
    k="-" if M is PZ else "GND"; d.line([(1365,M[k][1]),M[k]],fill=C["GND"],width=3)
# 5V bus at 1330
d.line([PR["5V"],(1330,PR["5V"][1]),(1330,R2["5V"][1])],fill=C["5V"],width=4)
for M in (R1,R2): d.line([(1330,M["5V"][1]),M["5V"]],fill=C["5V"],width=3)
# encoder signals
for M,pins in ((E1,("4   ENC1 CLK","5   ENC1 DT","6   ENC1 SW")),(E2,("15  ENC2 CLK","17  ENC2 DT","18  ENC2 SW"))):
    for lab,pin,xm in zip(("CLK (SIA)","DT (SIB)","SW"),pins,(1100,1130,1160)):
        wire(PR[pin],M[lab],"ENC",xm)
# ring data with resistors
def resistor(x,y,label):
    d.rectangle((x-22,y-7,x+22,y+7),fill="#f7e0b0",outline="black",width=2); d.text((x-26,y+10),label,font=F(12),fill="black")
wire(PR["47  RING1 DIN"],R1["DIN"],"DATA",1200); resistor(1270,R1["DIN"][1],"220 ohm")
wire(PR["21  RING2 DIN"],R2["DIN"],"DATA",1230); resistor(1270,R2["DIN"][1],"220 ohm")
# capacitors across 5V/GND at each ring
def cap(x,y5,yg,label):
    d.line([(x,y5),(x,(y5+yg)/2-5)],fill="black",width=3); d.line([(x,(y5+yg)/2+5),(x,yg)],fill="black",width=3)
    d.line([(x-14,(y5+yg)/2-5),(x+14,(y5+yg)/2-5)],fill="black",width=4); d.line([(x-14,(y5+yg)/2+5),(x+14,(y5+yg)/2+5)],fill="black",width=4)
    d.text((x+18,(y5+yg)/2-8),label,font=F(12),fill="black"); d.text((x-26,y5-4),"+",font=FB(13),fill="#d62728")
cap(1420,R1["5V"][1],R1["GND"][1],"1000 uF  (+ to 5V)")
cap(1420,R2["5V"][1],R2["GND"][1],"1000 uF  (+ to 5V)")
# piezo
wire(PR["7   PIEZO"],PZ["+"],"PZ",1260)

# legend / notes
y=1180
notes=["Shared: SCK, MOSI, DC, RST go to both TFTs; only CS differs (10 and 16).",
       "I2C: both OLEDs on SDA 8 / SCL 9. OLED 2 must be set to 0x3D (move the address resistor on its back, or bridge the pad).",
       "Rings: each has its own data pin (47, 21) with a 220 ohm in series close to the ring, and a 1000 uF across 5V/GND at the ring (stripe = -).",
       "Encoders: 3V3 and GND, CLK/DT/SW straight to the GPIOs (internal pull-ups). SmartElex pin order is SIA SIB SW GND VCC: wire by label.",
       "Never use GPIO 0, 3, 45, 46 (strapping), 19, 20 (USB), 26 to 37 (flash and PSRAM). Keep SPI leads under 15 cm."]
for n in notes: d.text((60,y),n,font=F(14),fill="#222"); y+=24
img.save("/mnt/user-data/outputs/desk_plane_radar_v6/wiring_v6.png"); print("ok")
