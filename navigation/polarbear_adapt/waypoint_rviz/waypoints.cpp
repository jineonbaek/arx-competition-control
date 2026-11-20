#include <cmath>
#include <utility>
#include <memory>
#include "point_inputs.hpp"
#include "route_list.hpp"
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include <QProcess>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QMetaObject>
#include <QComboBox>
#include <QListWidget>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QSignalBlocker>
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

// RViz may be started through systemd without the session's ARX_* environment.
// The viewer exports only the launcher path reliably; resolve the active metadata
// from that installation rather than guessing from the route definition.
struct DesktopSession {
  QString mode;
  bool competition = false;
};
static DesktopSession desktopSession() {
  DesktopSession session{qEnvironmentVariable("ARX_DESKTOP_MODE"), false};
  const auto launcher = qEnvironmentVariable("ARX_DESKTOP_LAUNCHER");
  if (launcher.isEmpty()) return session;
  QDir root = QFileInfo(launcher).absoluteDir();
  if (!root.cdUp()) return session;
  QFile active(root.filePath("reports/active-polarbear-adapt.txt"));
  if (!active.open(QIODevice::ReadOnly)) return session;
  const auto runPath = QString::fromUtf8(active.readAll()).trimmed();
  if (runPath.isEmpty()) return session;
  const QDir run(QDir::isAbsolutePath(runPath) ? runPath : root.filePath(runPath));
  session.competition = QFileInfo(run.filePath("competition-map")).isFile();
  QFile mode(run.filePath("desktop-mode"));
  // Once an active run is available, its mode takes precedence over stale env.
  session.mode = mode.open(QIODevice::ReadOnly) ? QString::fromUtf8(mode.readAll()).trimmed() : QString();
  return session;
}
static bool definitionTestMode(const QJsonObject &definition) {
  for (const auto value : definition["points"].toArray()) {
    if (value.toObject().contains("test_task_id")) return true;
  }
  return false;
}

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

class EndNavigation : public rviz_common::Tool {
public:
  void onInitialize() override {
    setName("结束当前导航任务");
    setDescription("结束当前单点或多点导航，保留所有路线点");
    auto node=context_->getRosNodeAbstraction().lock()->get_raw_node();
    cancel_=node->create_publisher<Empty>("/pb/drive_cancel",5);
  }
  void activate() override {
    cancel_->publish(Empty{});
    // Return to the normal interaction tool without requiring a map click.
    QTimer::singleShot(0,this,[this]{
      auto tools=context_->getToolManager();
      if(tools->getCurrentTool()==this)tools->setCurrentTool(tools->getDefaultTool());
    });
  }
  void deactivate() override {}
private:
  rclcpp::Publisher<Empty>::SharedPtr cancel_;
};

class MappingPanel : public rviz_common::Panel {
public:
  void onInitialize() override {
    auto layout = new QVBoxLayout(this);
    auto status = new QLabel();
    status->setWordWrap(true);
    QPushButton *direction = nullptr;
    for (auto entry : {std::pair<const char *, const char *>("保存比赛地图", "save-competition"),
                      std::pair<const char *, const char *>("保存比赛路线", "save-competition-route"),
                      std::pair<const char *, const char *>("用当前地图导航", "current-map"),
                      std::pair<const char *, const char *>("设定建图方向（备份并重建）", "set-map-direction")}) {
      auto button = new QPushButton(QString::fromUtf8(entry.first));
      layout->addWidget(button);
      const auto mode = QString::fromLatin1(entry.second);
      if (mode == "set-map-direction") {
        direction = button;
        button->setEnabled(desktopSession().mode == "mapping");
      }
      connect(button, &QPushButton::clicked, this, [this,status, mode] {
        if (mode == "set-map-direction" && desktopSession().mode != "mapping") {
          status->setText("仅纯建图模式可设定建图方向。");
          return;
        }
        if (mode == "set-map-direction" && QMessageBox::question(this,"设定建图方向",
            "请将车头与场地长边平行并保持静止。将先备份当前图，再按车头为X轴重新建图；旧路线不能沿用。继续？") != QMessageBox::Yes) return;
        const auto launcher = qEnvironmentVariable("ARX_DESKTOP_LAUNCHER");
        if (launcher.isEmpty() || !QProcess::startDetached(launcher, {mode})) {
          status->setText("无法打开建图启动器。");
          return;
        }
        status->setText("正在检查链路和保存地图，请留意系统提示。");
      });
    }
    auto modeTimer = new QTimer(this);
    connect(modeTimer, &QTimer::timeout, this, [direction] {
      if (direction) direction->setEnabled(desktopSession().mode == "mapping");
    });
    modeTimer->start(1000);
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
    },[this]{replaceCurrentPoint();});
    list_=new QListWidget();list_->setMinimumHeight(76);list_->setMaximumHeight(112);list_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Preferred);
    connect(list_,&QListWidget::currentItemChanged,this,[this]{inputs_->replace->setEnabled(!selectedPointId(list_).isEmpty());});
    auto actions=new QHBoxLayout();auto up=new QPushButton("上移");auto down=new QPushButton("下移");auto remove=new QPushButton("删除");
    for(auto button:{up,down}){button->setAutoRepeat(true);button->setAutoRepeatDelay(350);button->setAutoRepeatInterval(150);}
    actions->addWidget(up);actions->addWidget(down);actions->addWidget(remove);
    auto files=new QHBoxLayout();auto save=new QPushButton("保存路线");auto load=new QPushButton("载入为草稿");
    testMode_=new QPushButton("测试模式");testMode_->setCheckable(true);
    files->addWidget(save);files->addWidget(load);files->addWidget(testMode_);
    auto start=new QPushButton("开始路线");auto clear=new QPushButton("取消并清空");state_=new QLabel("等待导航执行器");state_->setWordWrap(true);
    layout->addWidget(list_);layout->addLayout(actions);layout->addLayout(files);auto execution=new QHBoxLayout();execution->addWidget(start);execution->addWidget(clear);layout->addLayout(execution);layout->addWidget(state_);layout->addStretch(1);
    auto refreshSession = [save,load,start] {
      const auto session = desktopSession();
      save->setText(session.competition ? "保存比赛路线" : "保存路线");
      load->setText(session.competition ? "重加载" : "载入为草稿");
      start->setEnabled(session.mode != "mapping");
      start->setToolTip(session.mode == "mapping" ? "纯建图模式仅可编辑路线，请切换导航后开始。" : "");
    };
    refreshSession();
    auto sessionTimer = new QTimer(this);
    connect(sessionTimer,&QTimer::timeout,this,refreshSession);sessionTimer->start(1000);
    connect(start,&QPushButton::clicked,this,[this]{
      if(desktopSession().mode == "mapping"){state_->setText("纯建图模式仅可编辑路线。");return;}
      start_->publish(Empty{});
    });
    connect(testMode_,&QPushButton::clicked,this,[this](bool enabled){
      // The definition is authoritative, including when the executor rejects an
      // edit while a route is running. Do not leave an optimistic check mark.
      const QSignalBlocker blocker(testMode_);
      testMode_->setChecked(definitionTestMode(definition_));
      send(QJsonObject{{"action","test_mode"},{"enabled",enabled}});
    });
    connect(clear,&QPushButton::clicked,this,[this]{clear_->publish(Empty{});});
    connect(remove,&QPushButton::clicked,this,[this]{int i=list_->currentRow();if(i>=0)send(QJsonObject{{"action","delete"},{"index",i}});});
    connect(up,&QPushButton::clicked,this,[this]{auto request=moveSelectedPoint(list_,definition_,-1);if(!request.isEmpty())send(request);});
    connect(down,&QPushButton::clicked,this,[this]{auto request=moveSelectedPoint(list_,definition_,1);if(!request.isEmpty())send(request);});
    connect(save,&QPushButton::clicked,this,[this]{
      if(desktopSession().competition){launchCompetitionRoute("save-competition-route");return;}
      if(definition_.isEmpty())return;
      auto name=QFileDialog::getSaveFileName(this,"保存路线定义",QDir::homePath()+"/navigation-route.json","JSON (*.json)");if(name.isEmpty())return;
      QFile file(name);if(!file.open(QIODevice::WriteOnly)){state_->setText("保存失败");return;}
      file.write(QJsonDocument(definition_).toJson(QJsonDocument::Indented));
    });
    connect(load,&QPushButton::clicked,this,[this]{
      if(desktopSession().competition){
        if(QMessageBox::question(this,"重加载比赛路线","将用已保存的比赛路线替换当前未保存点位。继续？")!=QMessageBox::Yes)return;
        launchCompetitionRoute("reload-competition-route");return;
      }
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
        auto doc=QJsonDocument::fromJson(text.toUtf8());if(!doc.isObject())return;
        updateRouteList(list_,definition_,doc.object());definition_=doc.object();
        inputs_->replace->setEnabled(!selectedPointId(list_).isEmpty());
        {const QSignalBlocker blocker(testMode_);testMode_->setChecked(definitionTestMode(definition_));}
      },Qt::QueuedConnection);
    });
  }
private:
  void replaceCurrentPoint(){
    const auto id=selectedPointId(list_);if(id.isEmpty()){state_->setText("请先选择要替换的点。");return;}
    for(auto value:definition_["points"].toArray()){
      const auto point=value.toObject();if(point["id"]!=id)continue;
      send(QJsonObject{{"action","replace_current"},{"id",id},{"route_id",definition_["route_id"]},{"expected_point",point},
        {"request_id",QUuid::createUuid().toString(QUuid::WithoutBraces)}});return;
    }
  }
  void launchCompetitionRoute(const QString &action){
    const auto launcher=qEnvironmentVariable("ARX_DESKTOP_LAUNCHER");
    if(launcher.isEmpty() || !QProcess::startDetached(launcher,{action})){
      state_->setText("无法打开比赛路线启动器。");return;
    }
    state_->setText("正在检查比赛路线操作，请留意系统提示。");
  }
  void send(const QJsonObject &value){std_msgs::msg::String m;m.data=QJsonDocument(value).toJson(QJsonDocument::Compact).toStdString();edit_->publish(m);}
  QListWidget *list_=nullptr;QLabel *state_=nullptr;QPushButton *testMode_=nullptr;QJsonObject definition_;
  rclcpp::Publisher<Empty>::SharedPtr start_,clear_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr edit_,capture_;
  std::unique_ptr<PointInputs> inputs_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_,definitions_;
};
}
PLUGINLIB_EXPORT_CLASS(arx_waypoints::MappingPanel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arx_waypoints::Panel,rviz_common::Panel)
PLUGINLIB_EXPORT_CLASS(arx_waypoints::Add,rviz_common::Tool)

PLUGINLIB_EXPORT_CLASS(arx_waypoints::EndNavigation,rviz_common::Tool)
