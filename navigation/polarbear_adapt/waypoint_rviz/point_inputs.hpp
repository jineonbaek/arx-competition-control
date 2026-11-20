#pragma once
#include <functional>
#include <QComboBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QVBoxLayout>
#include <QJsonObject>
#include <QJsonArray>
#include <QUuid>
namespace arx_waypoints {
inline QJsonObject typedPoint(const QString&kind,const QString&name,double x,double y,double yaw){
 QJsonObject p;const bool task=kind=="point_1"||kind=="point_2";p["type"]=task?"task":kind;p["pose"]=QJsonArray{x,y,yaw};
 if(task)p["point_id"]=kind;if(!name.trimmed().isEmpty())p["name"]=name.trimmed();return p;
}
struct PointInputs {
 QComboBox*kind;QPushButton*capture,*drag,*replace;
 PointInputs(QVBoxLayout*layout,std::function<void(const QJsonObject&)>onCapture,std::function<void(const QString&,const QString&)>onDrag,std::function<void()>onReplace=[](){}){
  kind=new QComboBox();kind->addItem("途经点（连续通过）","via");kind->addItem("特殊点（相邻直线，忽略障碍）","special");
  kind->addItem("任务点1（交给任务处理）","point_1");kind->addItem("任务点2（交给任务处理）","point_2");
  capture=new QPushButton("添加当前点");drag=new QPushButton("拖箭头加点");replace=new QPushButton("替换");
  replace->setToolTip("用当前有效车位与朝向替换选中点，保留点的类型和名称");replace->setEnabled(false);
  capture->setToolTip("按所选类型记录当前有效位置与朝向，只加点不出发");
  layout->setContentsMargins(4,4,4,4);layout->setSpacing(4);
  layout->addWidget(kind);auto buttons=new QHBoxLayout();buttons->setSpacing(4);buttons->addWidget(capture);buttons->addWidget(drag);buttons->addWidget(replace);layout->addLayout(buttons);
  QObject::connect(capture,&QPushButton::clicked,capture,[this,onCapture]{
   onCapture(QJsonObject{{"request_id",QUuid::createUuid().toString(QUuid::WithoutBraces)},{"kind",kind->currentData().toString()}});
  });
  QObject::connect(replace,&QPushButton::clicked,replace,[onReplace]{onReplace();});
  QObject::connect(drag,&QPushButton::clicked,drag,[this,onDrag]{onDrag(kind->currentData().toString(),QString());});
 }
};
}
