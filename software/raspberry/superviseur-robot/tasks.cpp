/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_BATTERY 40
#define PRIORITY_RELOAD 23
#define PRIORITY_SEARCHROBOT 26

int WD; //if 1 Watchdog enabled else 0
Arena Arene;
bool existArena = false;
bool isImageFluxActive = false;
std::list<Position> RobotsPosition;

/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */


void Tasks::PeriodicReload(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    /////////////////////Fonctionnalité 11 //////////////////////:
    Message * msgSend;
    
    rt_sem_p(&sem_startwithwd, TM_INFINITE);
    
    rt_task_set_periodic(NULL, TM_NOW, rt_timer_ns2ticks(1000 * 1000 * 1000));
    while (1) {

        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.ReloadWD());
        rt_mutex_release(&mutex_robot);
        cout << "Periodic message => (";
        cout << msgSend->GetID();
        cout << ")" << endl;
    }
    ////////////////////// End fonctionnalité 11 ////////////////
}

void Tasks::PeriodicCheckBattery(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    //////////////// Fonctionnalité 13 ////////////////////////////
    Message * msg;
    bool rs;

    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 500 * 1000 * 1000);

    while (1) {
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);

        if (rs) {
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = robot.Write(robot.GetBattery());
            rt_mutex_release(&mutex_robot);

            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msg);
            rt_mutex_release(&mutex_monitor);

        }

    }

    ////////////////End Fonctionnalité 13 ////////////////////////
}

void Tasks::PeriodicSearchRobot(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    rt_task_set_periodic(NULL, TM_NOW, 100 * 1000 * 1000);
    rt_sem_p(&sem_periodicsearchrobot, TM_INFINITE);
    MessagePosition* msgPos;
    MessageImg * msgImg;
    while (1) {
        rt_task_wait_period(NULL);
        if (camera.IsOpen() && isImageFluxActive) {
            Img * Image = new Img(camera.Grab());
            msgImg = new MessageImg(MESSAGE_CAM_IMAGE, Image);

            RobotsPosition = Image->SearchRobot(Arene);
            if (!RobotsPosition.empty()) {
                Image->DrawRobot(RobotsPosition.front());
                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, RobotsPosition.front());

            } else {
                Position PosVide;
                msgPos = new MessagePosition(MESSAGE_CAM_POSITION, PosVide);
            }
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgPos);
            rt_mutex_release(&mutex_monitor);

            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);
            rt_mutex_release(&mutex_monitor);
        }

        /*



        
        Message * msgSend;

        while (1){
        
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend=robot.Write(robot.ReloadWD());
            rt_mutex_release(&mutex_robot);
            cout << "Periodic message => (";
            cout << msgSend->GetID();
            cout << ")" << endl;*/
    }
}

void Tasks::PeriodicImageCamera(void *arg) {

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;

    //////////////// Fonctionnalité 13 ////////////////////////////
    Message * msg;
    bool rs;

    rt_sem_p(&sem_barrier, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 1000 * 1000 * 1000);

    while (1) {
        rt_task_wait_period(NULL);

        if (camera.IsOpen() && isImageFluxActive) {
            Img * img = new Img(camera.Grab());
            if (existArena) {
                img->DrawArena(Arene);
            }
            MessageImg * msgImg = new MessageImg(MESSAGE_CAM_IMAGE, img);
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgImg);
            rt_mutex_release(&mutex_monitor);
        }

    }

    ////////////////End Fonctionnalité 15 ////////////////////////
}

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startwithwd, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_periodicsearchrobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_BATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_periodicReload, "th_periodicReload", 0, PRIORITY_RELOAD, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_camera, "th_camera", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_searchRobot, "th_searchRobot", 0, PRIORITY_SEARCHROBOT, 0)) {//PRIO A CHANGER
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }


    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::PeriodicCheckBattery, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_periodicReload, (void(*)(void*)) & Tasks::PeriodicReload, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_camera, (void(*)(void*)) & Tasks::PeriodicImageCamera, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_searchRobot, (void(*)(void*)) & Tasks::PeriodicSearchRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            //delete(msgRcv);
            /////////////////// Fonctionnalité 5 ////////////////////

            cout << "Lost Communication with monitor, trying to reconnect ... " << endl;

            monitor.AcceptClient(); // Wait for Client to connect

            cout << "Reconnected successfully !" << endl;

            //exit(-1); no longer
            //////////////End Fonctionnalité 5 ////////////////////////

        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            WD = 0;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            WD = 1;
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        } else if (msgRcv->CompareID(MESSAGE_CAM_OPEN)) { //fonctionnalité 14
            Message * msgResp;
            if (camera.Open() == 0) {
                cout << "Error: Camera did not open" << endl << flush;
                msgResp = new Message(MESSAGE_ANSWER_NACK);
            } else {
                msgResp = new Message(MESSAGE_ANSWER_ACK);
                isImageFluxActive = true;
            }
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgResp); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);
        } else if (msgRcv->CompareID(MESSAGE_CAM_CLOSE)) {
            Message * msgResp;
            if (camera.IsOpen()) {
                camera.Close();
                if (camera.IsOpen()) {
                    cout << "Error when closing the camera" << endl << flush;
                    msgResp = new Message(MESSAGE_ANSWER_NACK);
                } else {
                    cout << "Closing camera..." << endl << flush;
                    msgResp = new Message(MESSAGE_ANSWER_ACK);
                }
            } else {
                cout << "Camera is already closed!";
                msgResp = new Message(MESSAGE_ANSWER_ACK);
            }
            rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
            monitor.Write(msgResp); // The message is deleted with the Write
            rt_mutex_release(&mutex_monitor);

            //FONCT 17
        } else if (msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)) {
            isImageFluxActive = false;
            existArena = false;

            Img * Image = new Img(camera.Grab());
            Arene = Image->SearchArena();
            if (Arene.IsEmpty()) {
                Message * msgResp;
                msgResp = new Message(MESSAGE_ANSWER_NACK);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgResp); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
                isImageFluxActive = true;
            } else {
                Image->DrawArena(Arene);
                MessageImg * msgImg = new MessageImg(MESSAGE_CAM_IMAGE, Image);
                rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
                monitor.Write(msgImg); // The message is deleted with the Write
                rt_mutex_release(&mutex_monitor);
            }

        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)) {
            //Arene
            isImageFluxActive = true;
            existArena = true;

        } else if (msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)) {
            isImageFluxActive = true;
            existArena = false;
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)) {
            rt_sem_v(&sem_periodicsearchrobot);
        } else if (msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)) {
            rt_sem_p(&sem_periodicsearchrobot, TM_INFINITE);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    Message * msgSend;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {


        ///////////////////////Fonctionnalité 11 ////////////////////////
        rt_sem_p(&sem_startRobot, TM_INFINITE);


        /////////////Fonctionnalité 9/////////
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.Reset());
        rt_mutex_release(&mutex_robot);
        //////////////////////////////////////

        if (WD == 1) {
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithWD());
            rt_mutex_release(&mutex_robot);
            rt_sem_broadcast(&sem_startwithwd);
            cout << msgSend->GetID();
            cout << ")" << endl;

        } else {
            rt_sem_p(&sem_startRobot, TM_INFINITE);
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;
        }



        ////////////////////////End Fonctionnalité 11 //////////////////

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    unsigned int compteur = 0;
    Message * msgSend;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {


        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);

            cout << " move: " << cpMove;

            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(new Message((MessageID) cpMove));
            rt_mutex_release(&mutex_robot);
            /////////////////// Fonctionnalité 8 /////////////////////
            if ((msgSend->GetID() == MESSAGE_ANSWER_NACK)||(msgSend->GetID() == MESSAGE_ANSWER_ROBOT_ERROR)||(msgSend->GetID() == MESSAGE_ANSWER_ROBOT_TIMEOUT)||(msgSend->GetID() == MESSAGE_ANSWER_COM_ERROR)) {
                compteur++;
                cout << "compteur incremente" << endl;
            } else {
                compteur = 0;
                cout << "compteur remis a 0" << endl;
            }
            ////////////////// End Fonctionnalité 8 ///////////////////

            ////////////////// Fonctionnalité 9 //////////////////////
            if (compteur == 3) {
                //reset to initial state
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 0;
                rt_mutex_release(&mutex_robotStarted);

                compteur = 0;

                cout << "Lost Communication between Robot & Supervisor" << endl;
                //robot.Close(); 

            }

            ///////////////// End Fonctionnalité 9 ///////////////////
        }


        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message * msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message * Tasks::ReadInQueue(RT_QUEUE * queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}

