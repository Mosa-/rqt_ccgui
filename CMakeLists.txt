cmake_minimum_required(VERSION 2.8.3)

project(rqt_ccgui)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_cpp
  rqt_gui_py
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})


set(rqt_ccgui_SRCS
	src/rqt_ccgui/ccgui.cpp
)
set(rqt_ccgui_HDRS
	include/rqt_ccgui/ccgui.h
)

set(rqt_ccgui_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR}
)

set(rqt_ccgui_UIS
	src/rqt_ccgui/ccgui.ui
)

catkin_package(
  INCLUDE_DIRS include ${rqt_ccgui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rospy rqt_gui rqt_gui_cpp rqt_gui_py
  #DEPENDS system_lib
)
catkin_python_setup()

qt4_wrap_cpp(rqt_ccgui_MOCS ${rqt_ccgui_HDRS})
qt4_wrap_ui(rqt_ccgui_UIS_H ${rqt_ccgui_UIS})

include_directories(${rqt_ccgui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_ccgui_SRCS} ${rqt_ccgui_MOCS} ${rqt_ccgui_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY resource
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS scripts/rqt_ccgui
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_ccgui
DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)



