"""Select the real T2 test pair without modifying the saved route."""

def t2_pair_indices(points):
 if not isinstance(points,list) or any(not isinstance(p,dict) for p in points):
  raise ValueError('T2实测路线点格式无效')
 matches=[i for i,p in enumerate(points) if p.get('type')=='task' and p.get('point_id')=='point_2']
 if len(matches)!=1:
  raise ValueError('T2实测需要唯一一个正式T2任务点')
 end=matches[0]
 if (end==0
     or points[end-1].get('test_task_id') or points[end].get('test_task_id')):
  raise ValueError('T2实测需要T2前一点且两点均不带测试任务标记')
 if any(p.get('type')=='task' or p.get('test_task_id') for p in points[end+1:]):
  raise ValueError('T2后续仅支持导航点，不能包含其他正式或测试任务')
 return end-1,end
