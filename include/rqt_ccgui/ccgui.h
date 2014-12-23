#ifndef rqt_ccgui__my_plugin_H
#define rqt_ccgui__my_plugin_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_ccgui.h>
#include <QWidget>
#include <QStringList>
#include <QIcon>
#include <QMessageBox>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
using namespace std;

namespace rqt_ccgui {

class ccgui
  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  ccgui();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  //bool hasConfiguration() const;
  //void triggerConfiguration();
private:
  Ui_Form ui_;
  QWidget* widget_;
  void configureHelpButtons();
  void activeSpeechCommand(QString command, QString data);
  void deactiveSpeechCommand(QString command);
  void receiveSpeechCommand(const std_msgs::StringConstPtr& cmd);

private slots:
  void clickLabelhelpBtn_timeout();
  void clickLabelhelpBtn_Dsleeptime();
  void clickLabelhelpBtn_DexeCount();
  void clickLabelhelpBtn_Dmovespeed();
  void clickLabelhelpBtn_DaccFactor();
  void clickLabelhelpBtn_maxSpeed();
  void clickLabelhelpBtn_DtwistFactor();
  void clickLabelhelpBtn_DtwistSpeed();
  void clickLabelhelpBtn_DgripperStep();

};
} // namespace
#endif
