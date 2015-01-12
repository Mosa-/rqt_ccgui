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

  QIcon refresh_icon;
  std::string path = ros::package::getPath("rqt_ccgui")+"/img/redstatus.png";
  QString icon_path(path.c_str());
  refresh_icon.addFile(icon_path);

  ui_.status->setIcon(refresh_icon);
  ui_.status->setIconSize(QSize(21, 21));
  ui_.status->setStyleSheet("QPushButton{border: none;outline: none;}");

  ui_.robot_active->setIcon(refresh_icon);
  ui_.robot_active->setIconSize(QSize(21, 21));
  ui_.robot_active->setStyleSheet("QPushButton{border: none;outline: none;}");

  this->configureHelpButtons();
  lastCmd = "move";
  this->deactiveSpeechCommand();
  lastCmd = "turn";
  this->deactiveSpeechCommand();
  lastCmd = "look";
  this->deactiveSpeechCommand();
  lastCmd = "grasp";
  this->deactiveSpeechCommand();
  connect(ui_.helpBtn_timeout, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_timeout()));
  connect(ui_.helpBtn_Dsleeptime, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_Dsleeptime()));
  connect(ui_.helpBtn_DexeCount, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_DexeCount()));
  connect(ui_.helpBtn_Dmovespeed, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_Dmovespeed()));
  connect(ui_.helpBtn_DaccFactor, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_DaccFactor()));
  connect(ui_.helpBtn_maxSpeed, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_maxSpeed()));
  connect(ui_.helpBtn_DtwistFactor, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_DtwistFactor()));
  connect(ui_.helpBtn_DtwistSpeed, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_DtwistSpeed()));
  connect(ui_.helpBtn_DgripperStep, SIGNAL(clicked()), this, SLOT(clickLabelhelpBtn_DgripperStep()));
  connect(ui_.startRSCI, SIGNAL(clicked()), this, SLOT(rsciBtn()));

  sub = getNodeHandle().subscribe("/speech_control_interface/cmd", 10, &ccgui::receiveSpeechCommand, this);
  cpPub = getNodeHandle().advertise<std_msgs::Float32MultiArray>("/rqt_ccg/configparameter", 1);
  robotActive = false;
  appStatus = false;

  timer = new QTimer(widget_);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateGUI()));
  timer->start(500);
}

void ccgui::shutdownPlugin()
{
	sub.shutdown();
	delete timer;
}

void ccgui::updateGUI(){
	if(speechCmd){
		this->deactiveSpeechCommand();
		this->activeSpeechCommand();
		lastCmd = currentCmd;
		speechCmd = false;
	}
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

void ccgui::receiveSpeechCommand(const std_msgs::StringConstPtr& cmd){
	ROS_INFO("Received string: %s appStatus: %d", cmd->data.c_str(), appStatus);

	if(appStatus){
		string cmds = cmd->data;

		vector<string> tokens;
		stringstream mySstream(cmds);
		string temp;
		while (getline(mySstream, temp, ' ')) {
			tokens.push_back(temp);
		}

		QString command = QString::fromStdString(tokens.at(0));
		QString dataToCmd = QString::fromStdString(tokens.at(1));


		if(command == "robo" || command == "move" || command == "turn" || command == "look" || command == "grasp"){
			speechCmd = true;
			currentCmd = command;
			currentData = dataToCmd;
		}
	}
}


void ccgui::configureHelpButtons(){
	  QIcon refresh_icon;
	  std::string path = ros::package::getPath("rqt_ccgui")+"/img/helpbtn.png";
	  QString icon_path(path.c_str());
	  refresh_icon.addFile(icon_path);

	  ui_.helpBtn_timeout->setIcon(refresh_icon);
	  ui_.helpBtn_timeout->setIconSize(QSize(21, 21));
	  ui_.helpBtn_timeout->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_Dsleeptime->setIcon(refresh_icon);
	  ui_.helpBtn_Dsleeptime->setIconSize(QSize(21, 21));
	  ui_.helpBtn_Dsleeptime->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_DexeCount->setIcon(refresh_icon);
	  ui_.helpBtn_DexeCount->setIconSize(QSize(21, 21));
	  ui_.helpBtn_DexeCount->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_Dmovespeed->setIcon(refresh_icon);
	  ui_.helpBtn_Dmovespeed->setIconSize(QSize(21, 21));
	  ui_.helpBtn_Dmovespeed->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_DaccFactor->setIcon(refresh_icon);
	  ui_.helpBtn_DaccFactor->setIconSize(QSize(21, 21));
	  ui_.helpBtn_DaccFactor->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_maxSpeed->setIcon(refresh_icon);
	  ui_.helpBtn_maxSpeed->setIconSize(QSize(21, 21));
	  ui_.helpBtn_maxSpeed->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_DtwistFactor->setIcon(refresh_icon);
	  ui_.helpBtn_DtwistFactor->setIconSize(QSize(21, 21));
	  ui_.helpBtn_DtwistFactor->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_DtwistSpeed->setIcon(refresh_icon);
	  ui_.helpBtn_DtwistSpeed->setIconSize(QSize(21, 21));
	  ui_.helpBtn_DtwistSpeed->setStyleSheet("QPushButton{border: none;outline: none;}");

	  ui_.helpBtn_DgripperStep->setIcon(refresh_icon);
	  ui_.helpBtn_DgripperStep->setIconSize(QSize(21, 21));
	  ui_.helpBtn_DgripperStep->setStyleSheet("QPushButton{border: none;outline: none;}");

}

void ccgui::activeSpeechCommand(){

	QLabel* label = NULL;
	QLabel* labelData = NULL;
	if(currentCmd == "move"){
		label = ui_.label_move;
		labelData = ui_.label_move_data;
	}else if(currentCmd == "turn"){
		label = ui_.label_turn;
		labelData = ui_.label_turn_data;
	}else if(currentCmd == "look"){
		label = ui_.label_look;
		labelData = ui_.label_look_data;
	}else if(currentCmd == "grasp"){
		label = ui_.label_gripper;
		labelData = ui_.label_grip_data;
	}else if(currentCmd == "robo"){
		if(currentData == "stop"){
			robotActive = false;
		}else if(currentData == "start"){
			robotActive = true;
		}
		changeLEDStatus("robot", robotActive);
	}

	if(label && labelData && robotActive){
		label->setStyleSheet("QLabel{ gridline-color: rgb(0, 0, 0); "
				"color: rgb(0,0,0); "
				"padding: 2px; background-color: rgb(160, 160, 160);"
				"border: 4px outset grey; } ");

		labelData->setText(currentData);
		labelData->setStyleSheet("QLabel{font: bold; font-size:24pt; color: rgb(0,0,204);}");
	}
}

void ccgui::deactiveSpeechCommand(){
	QLabel* label = NULL;
	QLabel* labelData = NULL;
	if(lastCmd == "move"){
		label = ui_.label_move;
		labelData = ui_.label_move_data;
	}else if(lastCmd == "turn"){
		label = ui_.label_turn;
		labelData = ui_.label_turn_data;
	}else if(lastCmd == "look"){
		label = ui_.label_look;
		labelData = ui_.label_look_data;
	}else if(lastCmd == "grasp"){
		label = ui_.label_gripper;
		labelData = ui_.label_grip_data;
	}

	if(label && labelData && robotActive){
		label->setStyleSheet("QLabel{ gridline-color: rgb(0, 0, 0); "
				"color: rgb(64,64,64); "
				"padding: 2px; background-color: rgb(96, 96, 96);"
				"border: 4px inset grey; } ");
		labelData->setStyleSheet("QLabel{ font-size:24pt; color: rgb(64,64,64);}");
	}
}

void ccgui::rsciBtn(){
	if(appStatus){
		ui_.startRSCI->setText("Start RSCI");
	}else{
		std_msgs::Float32MultiArray cpArr;
		cpArr.data.clear();
		cpArr.data.push_back(ui_.lineEdit_timeout->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_Dsleeptime->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DexeCount->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DmoveSpeed->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DaccFactor->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_maxSpeed->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DtwistFactor->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DtwistSpeed->text().toFloat());
		cpArr.data.push_back(ui_.lineEdit_DgripperStep->text().toFloat());

		cpPub.publish(cpArr);
		ui_.startRSCI->setText("Stop RSCI");
	}

	appStatus = !appStatus;
	changeLEDStatus("appStatus", appStatus);

}

void ccgui::changeLEDStatus(QString object, bool active){

	std::string status = "";
	QPushButton* btn = NULL;

	if(active){
		status = "greenstatus";
	}else{
		status = "redstatus";
	}

	if(object == "robot"){
		btn = ui_.robot_active;
	}else if(object == "appStatus"){
		btn = ui_.status;
	}
	  QIcon refresh_icon;
	  std::string path = ros::package::getPath("rqt_ccgui")+"/img/"+status+".png";
	  QString icon_path(path.c_str());
	  refresh_icon.addFile(icon_path);
	  btn->setIcon(refresh_icon);
	  btn->setIconSize(QSize(21, 21));
	  btn->setStyleSheet("QPushButton{border: none;outline: none;}");
}


void ccgui::clickLabelhelpBtn_timeout(){
	 int ret = QMessageBox::information(widget_, QString("About timeout"),
			 QString("Timeout between two commands.\n"
	                                   "in [ms]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_Dsleeptime(){
	 int ret = QMessageBox::information(widget_, QString("About default sleep time"),
			 QString("The default sleep time between two execution steps.\n"
	                                   "in [s]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_DexeCount(){
	 int ret = QMessageBox::information(widget_, QString("About default execution count"),
			 QString("The default execution count of a command.\n"
	                                   ),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_Dmovespeed(){
	 int ret = QMessageBox::information(widget_, QString("About default move speed"),
			 QString("The default move speed of the roboter.\n"
	                                   "in [m/s]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_DaccFactor(){
	 int ret = QMessageBox::information(widget_, QString("About default accelerator factor"),
			 QString("Describes the additional speed, if the roboter accelerate.\n"
	                                   "in [m/s]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_maxSpeed(){
	 int ret = QMessageBox::information(widget_, QString("About maximal speed"),
			 QString("The maximal speed of the roboter.\n"
	                                   "in [m/s]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_DtwistFactor(){
	 int ret = QMessageBox::information(widget_, QString("About default twist factor"),
			 QString("Describes the turn/twist factor of the roboter.\n"
	                                   "in ["+ QString::fromUtf8("°")+"]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_DtwistSpeed(){
	 int ret = QMessageBox::information(widget_, QString("About default twist speed"),
			 QString("The default twist/turn speed of the roboter.\n"
	                                   "in [rad/s]"),
	                                QMessageBox::Close);
}
void ccgui::clickLabelhelpBtn_DgripperStep(){
	 int ret = QMessageBox::information(widget_, QString("About default gripper step."),
			 QString("The default gripper step of the roboter arm.\n"
	                                   ),
	                                QMessageBox::Close);
}

} // namespace
PLUGINLIB_DECLARE_CLASS(rqt_ccgui, ccgui, rqt_ccgui::ccgui, rqt_gui_cpp::Plugin)
