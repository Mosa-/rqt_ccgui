#include "rqt_ccgui/ccgui.h"
#include <pluginlib/class_list_macros.h>


namespace rqt_ccgui {

ccgui::ccgui()
  : rqt_gui_cpp::Plugin()
  , widget_(0)
{
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("rqt_ccgui");
}

void ccgui::initPlugin(qt_gui_cpp::PluginContext& context)
{
  // access standalone command line arguments
  QStringList argv = context.argv();
  // create QWidget
  widget_ = new QWidget();
  // extend the widget with all attributes and children from UI file
  ui_.setupUi(widget_);
  // add widget to the user interface
  context.addWidget(widget_);

  this->configureHelpButtons();
}

void ccgui::shutdownPlugin()
{
  // TODO unregister all publishers here
}

void ccgui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void ccgui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
{
  return true;
}

void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

void ccgui::configureHelpButtons(){
	  QIcon refresh_icon;
	  std::string path = ros::package::getPath("rqt_ccgui")+"/img/helpbtn.png";
	  QString icon_path(path.c_str());
	  refresh_icon.addFile(icon_path);

	  ui_.helpBtn1->setIcon(refresh_icon);
	  ui_.helpBtn1->setIconSize(QSize(21, 21));
	  ui_.helpBtn1->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn2->setIcon(refresh_icon);
	  ui_.helpBtn2->setIconSize(QSize(21, 21));
	  ui_.helpBtn2->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn3->setIcon(refresh_icon);
	  ui_.helpBtn3->setIconSize(QSize(21, 21));
	  ui_.helpBtn3->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn4->setIcon(refresh_icon);
	  ui_.helpBtn4->setIconSize(QSize(21, 21));
	  ui_.helpBtn4->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn5->setIcon(refresh_icon);
	  ui_.helpBtn5->setIconSize(QSize(21, 21));
	  ui_.helpBtn5->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn6->setIcon(refresh_icon);
	  ui_.helpBtn6->setIconSize(QSize(21, 21));
	  ui_.helpBtn6->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn7->setIcon(refresh_icon);
	  ui_.helpBtn7->setIconSize(QSize(21, 21));
	  ui_.helpBtn7->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn8->setIcon(refresh_icon);
	  ui_.helpBtn8->setIconSize(QSize(21, 21));
	  ui_.helpBtn8->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn9->setIcon(refresh_icon);
	  ui_.helpBtn9->setIconSize(QSize(21, 21));
	  ui_.helpBtn9->setStyleSheet("QPushButton{border: none;outline: none;}");

}

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_ccgui, ccgui, rqt_ccgui::ccgui, rqt_gui_cpp::Plugin)
