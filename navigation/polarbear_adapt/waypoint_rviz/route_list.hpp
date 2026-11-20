#pragma once
#include <QListWidget>
#include <QJsonObject>
#include <QJsonArray>
#include <QSignalBlocker>
namespace arx_waypoints {
inline QString selectedPointId(const QListWidget *list) {
 return list->currentItem()?list->currentItem()->data(Qt::UserRole).toString():QString();
}
inline void updateRouteList(QListWidget *list,const QJsonObject&before,const QJsonObject&definition) {
 const QString selected=before["route_id"]==definition["route_id"]?selectedPointId(list):QString();
 const QSignalBlocker blocker(list);list->clear();int i=0,selectedRow=-1;
 for(auto value:definition["points"].toArray()) {
  auto p=value.toObject();auto pose=p["pose"].toArray();if(pose.size()!=3)continue;
  auto type=p["type"].toString();QString label=type=="task"?p["point_id"].toString():(type=="via"?"途经":type=="special"?"特殊直线":"旧版停车");
  auto item=new QListWidgetItem(QString("%1  %2  (%3, %4)  %5°").arg(++i).arg(label).arg(pose[0].toDouble(),0,'f',2).arg(pose[1].toDouble(),0,'f',2).arg(pose[2].toDouble()*180./3.141592653589793,0,'f',0),list);
  const auto id=p["id"].toString();item->setData(Qt::UserRole,id);
  if(!selected.isEmpty() && selected==id)selectedRow=list->count()-1;
 }
 list->setCurrentRow(selectedRow);
}
inline QJsonObject moveSelectedPoint(const QListWidget *list,const QJsonObject&definition,int delta) {
 const auto id=selectedPointId(list);if(id.isEmpty())return {};
 return QJsonObject{{"action","move"},{"id",id},{"delta",delta},{"route_id",definition["route_id"]}};
}
}
