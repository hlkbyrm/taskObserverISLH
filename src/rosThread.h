#include <ros/ros.h>
#include <QTimer>
#include <QVector>
#include <QThread>
#include <QObject>
#include <QTime>
#include <QString>
#include <QtCore/QString>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <ISLH_msgs/newTaskInfoMessage.h>
#include "std_msgs/UInt8.h"
#include <QUuid>

// task properties
struct taskProp{
  QString taskID;
  int handlingDuration; // in seconds - "the required time to handle the task"
  int timeOutDuration; // in seconds - "the timed-out duration for the task"
  QString requiredResources;
};

class RosThread:public QObject
{
    Q_OBJECT

public:

    RosThread();


private:
     bool shutdown;

     ros::NodeHandle n;

     ros::Publisher messageNewTaskInfoPub;

     ros::Subscriber messageTaskObserveOKSub;

     bool taskObserveOK;
     double taskLambda;

     int numTasks;

     QVector <taskProp> tasksList;

     bool readConfigFile(QString filename);

     void handleTaskObserveOK(std_msgs::UInt8 msg);

public slots:
     void work();

     void shutdownROS();


signals:
   void rosFinished();
   void rosStarted();
   void rosStartFailed();

};
