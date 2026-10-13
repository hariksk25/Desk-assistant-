import sys, os; sys.path.insert(0,'.'); import make_case_v6 as C, trimesh, numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Wedge, Polygon as MP
car=trimesh.load('stl/v6_carrier_white.stl'); cov=trimesh.load('stl/v6_cover_black.stl'); body=trimesh.load('stl/v6_body.stl')
bez=trimesh.load('stl/v6_bezel_ring_black.stl')
fig=plt.figure(figsize=(16,10),facecolor='#14161a')
# --- front view
ax=fig.add_axes([0.02,0.42,0.6,0.55]); ax.set_facecolor('#14161a'); ax.set_aspect('equal'); ax.axis('off')
def draw_sec(ax,m,z,col,zo):
    s=m.section(plane_origin=(0,0,z),plane_normal=(0,0,1))
    if s is None: return
    P,_=s.to_2D(to_2D=np.eye(4))
    from matplotlib.path import Path; from matplotlib.patches import PathPatch
    for poly in P.polygons_full:
        rings=[np.array(poly.exterior.coords)]+[np.array(h.coords) for h in poly.interiors]
        verts=np.concatenate(rings); codes=np.concatenate([[Path.MOVETO]+[Path.LINETO]*(len(r)-2)+[Path.CLOSEPOLY] for r in rings])
        ax.add_patch(PathPatch(Path(verts,codes),fc=col,ec='none',zorder=zo))
ax.add_patch(Rectangle((-5,-5),195,115,fc='#0b0c0e',zorder=0)); draw_sec(ax,car,2.9,'#e9e9e4',1)
for (cx,cy) in (C.POD1,C.POD2):
    for k in range(24):
        a=np.pi/2-2*np.pi*k/24; ax.add_patch(Circle((cx+39.2*np.cos(a),cy+39.2*np.sin(a)),2.6,fc='#ffb84d',alpha=0.55,zorder=2))
draw_sec(ax,cov,4.3,'#222428',3)
for (cx,cy) in (C.POD1,C.POD2):
    m=bez.copy(); m.apply_translation((cx,cy,C.PLATE_T)); draw_sec(ax,m,C.PLATE_T+0.5,'#2b2d32',4)
for (dx,dy) in C.DISP: ax.add_patch(Circle((dx,dy),C.TFT_WIN_D/2,fc='#0d2a3a',ec='#3fa9f5',lw=1,zorder=5)); ax.text(dx,dy,'round TFT\n(centred)',color='#9fd3ff',ha='center',va='center',fontsize=9,zorder=6)
for (sx,sy) in C.OLED_SCR: ax.add_patch(Rectangle((sx-C.OLED91_WIN[0]/2,sy-C.OLED91_WIN[1]/2),*C.OLED91_WIN,fc='#000',ec='#ffd23f',lw=1,zorder=5)); ax.text(sx,sy,'0.96" OLED',color='#ffd23f',ha='center',va='center',fontsize=8,zorder=6)
ax.set_xlim(-5,185); ax.set_ylim(-5,105); ax.set_title('Front (assembled, 180 x 100 mm)',color='w')
# --- section through pod 1 centre (x = POD1) : y-z
ax2=fig.add_axes([0.62,0.42,0.37,0.55]); ax2.set_facecolor('#14161a'); ax2.set_aspect('equal')
cx,cy=C.POD1
for m,col in ((body,'#6a8fc4'),(car,'#e9e9e4'),(cov,'#222428')):
    s=m.section(plane_origin=(cx+0.01,0,0),plane_normal=(1,0,0))
    if s is None: continue
    for e in s.discrete: ax2.plot(e[:,1],e[:,2],color=col,lw=1)
zr0,zr1=C.RING_SEAT_Z-C.RING_PCB_T,C.RING_SEAT_Z
for y0 in (cy-C.RING_OD/2,cy+C.RING_ID/2): ax2.add_patch(Rectangle((y0,zr0),(C.RING_OD-C.RING_ID)/2,C.RING_PCB_T,fc='#2e7d32',zorder=3))
for yc in (cy-39.2,cy+39.2): ax2.add_patch(Rectangle((yc-2.5,zr1),5,C.RING_LED_H,fc='#ffb84d',zorder=3))
ox,oy=C.OLEDS[0]; ax2.add_patch(Rectangle((oy-C.OLED91_BOARD[1]/2,zr0-C.OLED91_BOARD[2]),C.OLED91_BOARD[1],C.OLED91_BOARD[2],fc='#1565c0',zorder=4))
sx,sy=C.OLED_GLASS[0]; ax2.add_patch(Rectangle((sy-C.OLED91_SCREEN[1]/2,zr0),C.OLED91_SCREEN[1],C.OLED91_GLASS_T,fc='#ffd23f',zorder=4))
dx,dy=C.DISP[0]; zp=C.GLASS_POCKET_T-C.TFT_FRONT_T; ax2.add_patch(Rectangle((dy-C.TFT_MOD_R,zp-C.TFT_PCB_T),C.TFT_MOD_R+C.TFT_TAB[1],C.TFT_PCB_T,fc='#3fa9f5',zorder=4)); ax2.add_patch(Rectangle((dy-17.8,zp),17.8+C.TFT_BL[1],C.TFT_FRONT_T,fc='#0d2a3a',ec='#3fa9f5',zorder=4))
ax2.set_xlim(cy-50,cy+50); ax2.set_ylim(-14,8); ax2.tick_params(colors='#aaa'); ax2.set_title('Section through the left dial (y vs z, front up)',color='w')
for t,y,z,c in (('ring PCB in pit',cy+30,-7,'#4caf50'),('OLED board on ring back',cy-30,-10,'#64b5f6'),('OLED glass',cy-27,3.6,'#ffd23f'),('TFT',cy+5,5,'#9fd3ff')): ax2.text(y,z,t,color=c,fontsize=8,ha='center')
# --- face-down stack
ax3=fig.add_axes([0.05,0.03,0.9,0.36]); ax3.axis('off'); ax3.set_facecolor('#14161a')
rows=[('1','Front unit: carrier face up, cover on it (2 x M2 x 8 + tape), bezels (black skirts down into the windows) + red plugs','#222428'),('2','Flip the front unit onto a cloth (3 mm diffuser plate + 3.7 mm back layer facing up)','#e9e9e4'),
('3','Round displays: glass into the recess, pin tab toward 12 o\'clock','#3fa9f5'),('4','LED rings: LEDs down into the pits, resting on the inner ledge, wire joint at 12 o\'clock','#4caf50'),
('5','0.96" OLEDs UPSIDE DOWN (pins at the bottom): glass into the slot, pin tips into their slot','#ffd23f'),('6','Clamp plates, body over everything, 4 x M3 x 35 from the back','#6a8fc4')]
for i,(n,t,c) in enumerate(rows):
    ax3.add_patch(Rectangle((0.0,0.84-i*0.16),0.03,0.12,fc=c,transform=ax3.transAxes)); ax3.text(0.045,0.9-i*0.16,f'{n}.  {t}',color='w',fontsize=12,va='center',transform=ax3.transAxes)
fig.savefig('renders/subdial_layout.png',dpi=90,facecolor=fig.get_facecolor())
