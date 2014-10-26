#include "rosThread.h"
#include <QDebug>
#include <qjson/parser.h>
#include <QDir>
#include <QFile>
#include <time.h>
#include <stdlib.h>


RosThread::RosThread()
{
    shutdown = false;

    taskObserveOK = false;

}



void RosThread::work(){

    if(!ros::ok()){

        emit this->rosFinished();

        return;
    }

    emit rosStarted();

    ros::Rate loop(1);

    QString path = QDir::homePath();
    path.append("/ISL_workspace/src/configISL.json");


    if(!readConfigFile(path)){

        qDebug()<< "Read Config File Failed!!!";

        ros::shutdown();

        emit rosFinished();
    }


    messageNewTaskInfoPub = n.advertise<ISLH_msgs::newTaskInfoMessage>("taskObserverISLH/newTaskInfo",queueSize);


    messageTaskObserveOKSub = n.subscribe("taskHandlerISLH/taskObserveOK", 1,&RosThread::handleTaskObserveOK, this);


    srand( time( NULL ) );


    long seed;
    gsl_rng *rng;
    rng = gsl_rng_alloc(gsl_rng_rand48);
    seed = time(NULL)*getpid();
    gsl_rng_set(rng, seed);
    int prn;

    while(ros::ok())
    {

        if (taskObserveOK==true)
        {
            prn = gsl_ran_poisson(rng, taskLambda);

            if(prn>=1)
            {
                std::time_t encounteringTime = std::time(0); // t is an integer type

                qDebug()<< " time " << encounteringTime;

                ISLH_msgs::newTaskInfoMessage newTaskMsg;

                newTaskMsg.timeStamp = encounteringTime;

                QString taskUUIDStr = QUuid::createUuid().toString();
                // Get rid of these characters {, -, }  in taskUUIDStr "{3ac5c31b-9b0e-4eef-a724-f9c2a3965320}"
                taskUUIDStr.remove("{");
                taskUUIDStr.remove("-");
                taskUUIDStr.remove("}");

                newTaskMsg.taskUUID = taskUUIDStr.toStdString();

                qDebug()<< " taskUUID " << taskUUIDStr;

                int taskIndx = round((rand() % (numTasks)));

                qDebug()<< " Task Indx " << taskIndx;

                newTaskMsg.handlingDuration = tasksList.at(taskIndx).handlingDuration;

                newTaskMsg.timeOutDuration = tasksList.at(taskIndx).timeOutDuration;

                newTaskMsg.requiredResources = tasksList.at(taskIndx).requiredResources.toStdString();

                messageNewTaskInfoPub.publish(newTaskMsg);
            }
        }


        ros::spinOnce();

        loop.sleep();

    }

    qDebug()<<"I am quitting";

    ros::shutdown();

    emit rosFinished();


}
void RosThread::shutdownROS()
{
    ros::shutdown();
    // shutdown = true;


}

// Reads the config file
bool RosThread::readConfigFile(QString filename)
{
    QFile file(filename);

    if(!file.exists()) return false;

    if(!file.open(QFile::ReadOnly)) return false;

    QJson::Parser parser;

    bool ok;

    QVariantMap result = parser.parse(&file,&ok).toMap();

    if(!ok){

        file.close();
        qDebug()<<"Fatal reading error";

        return false;
    }
    else
    {
        taskLambda = result["poissonTaskLambda"].toDouble();
        qDebug()<< " Task lambda " << taskLambda;

        queueSize = result["queueSize"].toInt();
        qDebug()<<result["queueSize"].toString();


        //int numTasks = result["numtasks"].toInt();

        //this->tasksResourceList.resize(numTasks);

        QVariantMap nestedMap = result["Tasks"].toMap();

        numTasks = 0;
        foreach (QVariant plugin, nestedMap["Task"].toList())
        {

            taskProp temp;

            temp.handlingDuration = plugin.toMap()["handlingDuration"].toInt();
            temp.timeOutDuration = plugin.toMap()["timeoutDuration"].toInt();
            temp.requiredResources = plugin.toMap()["resources"].toString();
            tasksList.append(temp);

            numTasks ++;
        }


    }
    file.close();
    return true;

}

void RosThread::handleTaskObserveOK(std_msgs::UInt8 msg)
{
    if (msg.data==1)
    {
        qDebug()<< " Observe tasks ";
        taskObserveOK = true;
    }
    else if (msg.data==0)
    {
        qDebug()<< " Do not observe tasks ";
        taskObserveOK = false;
    }
}
