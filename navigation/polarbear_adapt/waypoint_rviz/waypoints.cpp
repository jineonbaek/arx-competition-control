#include <cmath>
#include <utility>
#include <memory>
#include "point_inputs.hpp"
#include <QLabel>
#include <QPushButton>
#include <QProcess>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QComboBox>
#include <QListWidget>
#include <QFileDialog>
#include <QFile>
#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QMessageBox>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace arx_waypoints {
using Empty = std_msgs::msg::Empty;
class Add : public rviz_default_plugins::tools::PoseTool {
public:
  Add() { shortcut_key_ = 'w'; }
  void setPointProperties(const QString &kind,const QString &name) { kind_=kind;name_=name; }
  void onInitialize() override {
    PoseTool::onInitialize();setName("添加路线点");
    node_=context_->getRosNodeAbstraction().lock()->get_raw_node();
    pub_=node_->create_publisher<std_msgs::msg::String>("/pb/waypoints/add_typed",10);
  }
protected:
  void onPoseSet(double x,double y,double yaw) override {
    if(context_->getFixedFrame()!="map") {RCLCPP_WARN(node_->get_logger(),"Route points require map frame");return;}
    QJsonObject p=typedPoint(kind_,name_,x,y,yaw);
    std_msgs::msg::String m;m.data=QJsonDocument(p).toJson(QJsonDocument::Compact).toStdString();pub_->publish(m);
  }
private:
  QString kind_="via",name_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

class MappingPanel : public rviz_common::Panel {
public:
  void onInitialize() override {
    auto layout = new QVBoxLayout(this);
    auto status = new QLabel();
    status->setWordWrap(true);
    for (auto entry : {std::pair<const char *, const char *>("用当前地图导航", "current-map"),
                      std::pair<const char *, const char *>("保存并新建地图", "new-map")}) {
      auto button = new QPushButton(QString::fromUtf8(entry.first));
      layout->addWidget(button);
      const auto mode = QString::fromLatin1(entry.second);
      connect(button, &QPushButton::clicked, this, [status, mode] {
        const auto launcher = qEnvironmentVariable("ARX_DESKTOP_LAUNCHER");
        if (launcher.isEmpty() || !QProcess::startDetached(launcher, {mode})) {
          status->setText("无法打开建图启动器。");
          return;
        }
        status->setText("正在检查链路和保存地图，请留意系统提示。");
      });
    }
    layout->addWidget(status);
  }
};

class Panel : public rviz_common::Panel {
public:
  void onInitialize() override {
    auto node=getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    start_=node->create_publisher<Empty>("/pb/waypoints/start",5);
    clear_=node->create_publisher<Empty>("/pb/waypoints/clear",5);
    edit_=node->create_publisher<std_msgs::msg::String>("/pb/waypoints/edit",10);
    capture_=node->create_publisher<std_msgs::msg::String>("/pb/waypoints/add_current",10);
    auto layout=new QVBoxLayout(this);
    inputs_=std::make_unique<PointInputs>(layout,[this](const QJsonObject&v){std_msgs::msg::String m;m.data=QJsonDocument(v).toJson(QJsonDocument::Compact).toStdString();capture_->publish(m);},[this](const QString&kind,const QString&name){
      auto tools=getDisplayContext()->getToolManager();
      for(int i=0;i<tools->numTools();++i){auto tool=tools->getTool(i);
        if(tool->getClassId()=="arx_waypoints/Add"){
          auto typed=dynamic_cast<Add*>(tool);if(typed)typed->setPointProperties(kind,name);tools->setCurrentTool(tool);break;
        }
      }
    });
    list_=new QListWidget();list_->setMinimumHeight(76);list_->setMaximumHeight(112);list_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
    auto actions=new QHBoxLayout();auto up=new QPushButton("上移");auto down=new QPushButton("下移");auto remove=new QPushButton("删除");
    actions->addWidget(up);actions->addWidget(down);actions->addWidget(remove);
    auto files=new QHBoxLayout();auto save=new QPushButton("保存路线");auto load=new QPushButton("载入为草稿");files->addWidget(save);files->addWidget(load);
    auto start=new QPushButton("开始路线");auto clear=new QPushButton("取消并清空");state_=new QLabel("等待导航执行器");state_->setWordWrap(true);
    layout->addWidget(list_);layout->addLayout(actions);layout->addLayout(files);auto execution=new QHBoxLayout();execution->addWidget(start);execution->addWidget(clear);layout->addLayout(execution);layout->addWidget(state_);layout->addStretch(1);
    connect(start,&QPushButton::clicked,this,[this]{start_->publish(Empty{});});
    connect(clear,&QPushButton::clicked,this,[this]{clear_->publish(Empty{});});
    connect(remove,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>=0)send(QJsonObject{{"action","delete"},{"index",i}});});
    connect(up,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>0)send(QJsonObject{{"action","move"},{"index",i},{"to",i-1}});});
    connect(down,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>=0 && i+1<list_->count())send(QJsonObject{{"action","move"},{"index",i},{"to",i+1}});});
    connect(save,&QPushButton::clicked,this,[this]{
      if(definition_.isEmpty())return;
      auto name=QFileDialog::getSaveFileName(this,"保存路线定义",QDir::homePath()+"/navigation-route.json","JSON (*.json)");if(name.isEmpty())return;
      QFile file(name);if(!file.open(QIODevice::WriteOnly)){state_->setText("保存失败");return;}
      file.write(QJsonDocument(definition_).toJson(QJsonDocument::Indented));
    });
    connect(load,&QPushButton::clicked,this,[this]{
      auto name=QFileDialog::getOpenFileName(this,"载入路线为草稿",QDir::homePath(),"JSON (*.json)");if(name.isEmpty())return;
      QFile file(name);if(!file.open(QIODevice::ReadOnly) || file.size()>262144){state_->setText("路线文件无效或过大");return;}
      QJsonParseError error;auto doc=QJsonDocument::fromJson(file.readAll(),&error);
      if(error.error!=QJsonParseError::NoError || !doc.isObject()){state_->setText("路线JSON无效");return;}
      bool mismatch=doc.object()["map_id"]!=definition_["map_id"];
      if(mismatch && QMessageBox::question(this,"地图身份不同","坐标可能不适用于当前地图。只载入为草稿，手动核对后再开始？")!=QMessageBox::Yes)return;
      send(QJsonObject{{"action","load"},{"definition",doc.object()},{"allow_map_change",mismatch}});
    });
    status_=node->create_subscription<std_msgs::msg::String>("/pb/waypoints/status",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){
      const auto text=QString::fromStdString(m->data);QMetaObject::invokeMethod(this,[this,text]{state_->setText(text);},Qt::QueuedConnection);
    });
    definitions_=node->create_subscription<std_msgs::msg::String>("/pb/waypoints/definition",rclcpp::QoS(1).transient_local(),[this](std_msgs::msg::String::ConstSharedPtr m){
      const auto text=QString::fromStdString(m->data);QMetaObject::invokeMethod(this,[this,text]{
        auto doc=QJsonDocument::fromJson(text.toUtf8());if(!doc.isObject())return;definition_=doc.object();int row=list_->currentRow();list_->clear();int i=0;
        for(auto value:definition_["points"].toArray()){
          auto p=value.toObject();auto pose=p["pose"].toArray();if(pose.size()!=3)continue;
          auto type=p["type"].toString();QString label=type=="task"?p["point_id"].toString():(type=="via"?"途经":type=="special"?"特殊直线":"旧版停车");
          list_->addItem(QString("%1  %2  (%3, %4)  %5°").arg(++i).arg(label).arg(pose[0].toDouble(),0,'f',2).arg(pose[1].toDouble(),0,'f',2).arg(pose[2].toDouble()*180./3.141592653589793,0,'f',0));
        }
        if(row>=0 && row<list_->count())list_->setCurrentRow(row);
      },Qt::QueuedConnection);
    });
  }
private:
  void send(const QJsonObject &value){std_msgs::msg::String m;m.data=QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();edit_->publish(m);}
  QListWidget *list_=nullptr;QLabel *state_=nullptr;QJsonObject definition_;
  rclcpp::Publisher<Empty>::SharedPtr start_,clear_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr edit_,capture_;
  std::unique_ptr<PointInputs> inputs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_,definitions_;
};
}
PLUGINLIB_EXPORT_CLASS(arx_waypoints::MappingPanel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arx_waypoints::Panel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arx_waypoints::Add,rviz_common::Tool)
