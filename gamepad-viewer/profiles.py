"""Strict, measured controller profiles. Unknown signatures must never drive."""
import array
import fcntl
import os
from pathlib import Path

class UnknownMode(ValueError):
    pass

NS = dict(mode='NS', vendor=0x057e, product=0x2009,
          axis_codes=[0,1,3,4,16,17], button_codes=[304,305,307,308,309,310,311,312,313,314,315,316,317,318],
          centers=[-16380,16379,-16380,16379,0,0], low=[-32767,0,-32767,0,-32767,-32767], high=[0,32767,0,32767,32767,32767],
          logical=[0,1,2,3,4,5], triggers={},
          buttons={'Y':2,'B':1,'A':0,'X':3,'LB':5,'RB':6,'LT':7,'RT':8,'BACK':9,'START':10,'HOME':11,'L3':12,'R3':13,'M1':13,'M2':12})
XINPUT = dict(mode='XINPUT', vendor=0x20bc, product=0x5128,
              axis_codes=[0,1,2,3,4,5,16,17], button_codes=[304,305,307,308,310,311,314,315,316,317,318],
              centers=[0,0,-32767,0,0,-32767,0,0],
              low=[-32767,-32763,-32767,-31806,-32698,-32767,-32767,-32767],
              high=[32729,32603,32767,32331,32265,32767,32767,32767],
              logical=[0,1,3,4,6,7], triggers={'LT':2,'RT':5},
              buttons={'Y':3,'B':1,'A':0,'X':2,'LB':4,'RB':5,'BACK':6,'START':7,'HOME':8,'L3':9,'R3':10,'M1':1,'M2':4})
PROFILES = {'NS':NS, 'XINPUT':XINPUT}


def select_profile(vendor, product, axis_codes, button_codes):
    for profile in PROFILES.values():
        if (vendor,product,axis_codes,button_codes)==(profile['vendor'],profile['product'],profile['axis_codes'],profile['button_codes']):
            return profile
    raise UnknownMode(f'未知手柄模式或映射变化：USB {vendor:04x}:{product:04x}，轴 {axis_codes}，按键 {button_codes}；禁止底盘输出')


def inspect_device(path):
    fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK)
    try:
        ac=array.array('B',[0]);bc=array.array('B',[0]);am=array.array('B',[0]*64);bm=array.array('H',[0]*512)
        fcntl.ioctl(fd,0x80016a11,ac,True);fcntl.ioctl(fd,0x80016a12,bc,True)
        fcntl.ioctl(fd,0x80406a32,am,True);fcntl.ioctl(fd,0x84006a34,bm,True)
        dev=Path('/sys/class/input')/Path(os.path.realpath(path)).name/'device'
        vendor=int((dev/'id/vendor').read_text().strip(),16)
        product=int((dev/'id/product').read_text().strip(),16)
        name=(dev/'name').read_text().strip()
        profile=select_profile(vendor,product,list(am[:ac[0]]),list(bm[:bc[0]]))
        if 'BTP-KP20' not in name and not (profile['mode']=='XINPUT' and name=='Generic X-Box pad'):
            raise UnknownMode('未知手柄名称：'+name+'；禁止底盘输出')
        return fd,profile,name
    except Exception:
        os.close(fd)
        raise


def raw_normalized(profile,index,value):
    if index in profile['triggers'].values():
        return max(0.,min(1.,(value+32767)/65534))
    center=profile['centers'][index]
    span=profile['high'][index]-center if value>=center else center-profile['low'][index]
    return max(-1.,min(1.,(value-center)/span))


def logical_axes(profile,axes):
    return [raw_normalized(profile,i,axes[i]) for i in profile['logical']]


def lt_pressed(profile,axes,buttons):
    if 'LT' in profile['triggers']:
        return raw_normalized(profile,profile['triggers']['LT'],axes[profile['triggers']['LT']])>=.5
    return bool(buttons[profile['buttons']['LT']])
