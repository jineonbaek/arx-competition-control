"""Version the competition route with its map; loading never starts navigation."""
from pathlib import Path
import copy,json,os,sys,time,uuid,urllib.request,http.cookiejar
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'polarbear_adapt'))
from competition_map import locked,validate
from mission_route import MissionRoute

def _json(path):return json.loads(Path(path).read_text())
def coordinate_id(root,run,checkpoint=None,new=False):
    root,run=Path(root),Path(run);dest=run/'competition-coordinate-id'
    with locked(root):
        if dest.exists():return dest.read_text().strip()
        source=Path(checkpoint).parent if checkpoint else root/'maps/比赛地图'
        meta=source/'coordinate-frame.json'
        ident=_json(meta)['coordinate_id'] if not new and meta.is_file() else uuid.uuid4().hex
        if not isinstance(ident,str) or not ident:raise ValueError('Invalid coordinate identity')
        tmp=dest.with_name(dest.name+'.'+uuid.uuid4().hex);tmp.write_text(ident+'\n');os.replace(tmp,dest)
        return ident

def route_payload(definition,ident):
    from route_test_mode import normal_definition
    d=normal_definition(definition);d['map_id']='competition:'+ident
    MissionRoute(d['map_id']).load(d)
    return dict(name='比赛路线',coordinate_id=ident,definition=d)

def select_payload(definition,previous,ident):
    if definition is not None:return route_payload(definition,ident)
    if previous is not None and previous.get('coordinate_id')==ident:
        return route_payload(previous['definition'],ident)
    return None

def read_definition(folder,ident,map_id):
    p=_json(Path(folder)/'competition-route.json')
    if p['coordinate_id']!=ident:raise ValueError('比赛路线坐标版本与地图不一致；未加载旧路线')
    result=route_payload(p['definition'],ident)['definition'];result['map_id']=map_id
    MissionRoute(map_id).load(result);return result

def observer():
    op=urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))
    op.open('http://127.0.0.1:9983/map.html',timeout=5).read()
    def get():return json.load(op.open('http://127.0.0.1:9983/api/navigation-view',timeout=5))
    return get

def snapshot(root,folder,run):
    root,folder,run=Path(root),Path(folder),Path(run)
    ident=coordinate_id(root,run,new=(run/'initial-heading.json').exists())
    current=observer()()['route'];definition=None
    if current.get('available') and current.get('definition',{}).get('map_id')=='live:'+str(run):
        definition=current['definition']
    elif (run/'desktop-mode').read_text().strip()!='mapping':
        raise ValueError('当前导航路线尚未就绪，未覆盖比赛路线')
    previous=root/'maps/比赛地图/competition-route.json'
    old=_json(previous) if previous.is_file() else None
    payload=select_payload(definition,old,ident)
    (folder/'coordinate-frame.json').write_text(json.dumps(dict(coordinate_id=ident,resolution=.02),ensure_ascii=False,indent=2))
    if payload is not None:(folder/'competition-route.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False))
    return payload

def link_route(root):
    root=Path(root);routes=root/'routes';routes.mkdir(exist_ok=True);target=routes/'比赛路线.json'
    with locked(root):
        if target.exists() and not target.is_symlink():
            archive=root/'reports/competition-route-previous';archive.mkdir(exist_ok=True);target.rename(archive/(uuid.uuid4().hex+'.json'))
        tmp=routes/('.route-'+uuid.uuid4().hex);tmp.symlink_to('../maps/比赛地图/competition-route.json');os.replace(tmp,target)
    return target

def load(root,run,checkpoint,replace=False):
    import rclpy
    from std_msgs.msg import String
    root,run=Path(root),Path(run);folder=Path(checkpoint).parent
    if not (folder/'competition-route.json').exists():print('本版比赛地图没有保存路线，保持空草稿');return
    validate(folder);ident=coordinate_id(root,run,checkpoint)
    desired=read_definition(folder,ident,'live:'+str(run));get=observer()
    def available():
        v=get();r=v.get('route',{});task=v.get('competition',{}).get('navigation',{}).get('handoff',{}).get('route',{})
        status=r.get('drive_status','').split()
        if (task.get('running') or (not replace and not r.get('status','').startswith('待开始')) or
                (status and status[0] not in ('idle','ready','ready_for_new_rviz_arrow','mapping_editor_idle','cancelled','arrived_stopped'))):
            raise ValueError('当前导航非空闲草稿，不覆盖')
        return r
    end=time.monotonic()+10
    while True:
        r=get().get('route',{})
        if r.get('available') and r.get('definition',{}).get('map_id')==desired['map_id']:break
        if time.monotonic()>=end:raise ValueError('等待导航路线接口超时')
        time.sleep(.2)
    r=available()
    if not replace and r.get('definition',{}).get('points'):
        print('当前会话已有路线，保留当前编辑');return
    from rclpy.qos import QoSProfile,DurabilityPolicy
    rclpy.init();node=rclpy.create_node('load_competition_route_draft');pub=node.create_publisher(String,'/pb/waypoints/edit',10)
    received={'definition':None,'status':''}
    def definition_message(message):
        try:received['definition']=json.loads(message.data)
        except (ValueError,TypeError):pass
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String,'/pb/waypoints/definition',definition_message,qos)
    node.create_subscription(String,'/pb/waypoints/status',lambda m:received.update(status=m.data),qos)
    def editor_ready():
        # Unknown/stale participants must not satisfy the editor handshake.
        editors={'pb_follow_once','pb_mapping_route_editor'}
        return pub.get_subscription_count()>0 and any(
            info.node_name in editors for info in node.get_subscriptions_info_by_topic('/pb/waypoints/edit'))
    try:
        end=time.monotonic()+8;matched=None
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if editor_ready():
                if matched is None:matched=time.monotonic()
                if time.monotonic()-matched>=.3:break
            else:matched=None
        else:raise ValueError('没有就绪的正式路线编辑器，未修改点位')
        final=available();expected=copy.deepcopy(final['definition'])
        if not replace and expected.get('points'):raise ValueError('用户已编辑新路线，未覆盖')
        edit=(dict(action='replace_if_unchanged',definition=desired,expected_definition=expected) if replace else
              dict(action='load_if_empty',definition=desired,expected_route_id=expected['route_id']))
        payload=String(data=json.dumps(edit,ensure_ascii=False));attempts=0;last_sent=-float('inf')
        end=time.monotonic()+12
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            current=available().get('definition')
            if current==desired:
                print('比赛路线已载入为待开始：'+str(len(desired['points']))+'点；未发送start');return
            if current!=expected:
                raise ValueError('等待重加载期间路线已改变，保留用户点位，未继续覆盖')
            # Every resend retains the original compare-and-swap guard. Never
            # retarget retries to a newer user draft or clear before loading.
            if editor_ready() and received['definition']!=desired and time.monotonic()-last_sent>=1.:
                pub.publish(payload);attempts+=1;last_sent=time.monotonic()
        detail=received['status'] or '执行器未返回路线更新'
        raise ValueError('比赛路线载入未确认（发送'+str(attempts)+'次；'+detail+'）')
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    root=Path(__file__).resolve().parent.parent;command=sys.argv[1]
    if command=='init':print(coordinate_id(root,Path(sys.argv[2]),sys.argv[3] or None,sys.argv[4]=='true'))
    elif command=='snapshot':snapshot(root,sys.argv[2],sys.argv[3])
    elif command=='link':print(link_route(root))
    elif command in ('load','reload'):load(root,sys.argv[2],sys.argv[3],replace=command=='reload')
    else:raise SystemExit('Unknown route operation')
