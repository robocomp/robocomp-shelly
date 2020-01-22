/*
 *    Copyright (C) 2020 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */


/** \mainpage RoboComp::localizationAgent
 *
 * \section intro_sec Introduction
 *
 * The localizationAgent component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd localizationAgent
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/localizationAgent --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <agmcommonbehaviorI.h>
#include <agmexecutivetopicI.h>
#include <aprilbasedlocalizationI.h>
#include <cgrI.h>

#include <Planning.h>
#include <GenericBase.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class localizationAgent : public RoboComp::Application
{
public:
	localizationAgent (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void ::localizationAgent::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::localizationAgent::run(int argc, char* argv[])
{
	QCoreApplication a(argc, argv);  // NON-GUI application


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	LoggerPrx logger_pubproxy;
	AGMExecutivePrx agmexecutive_proxy;
	OmniRobotPrx omnirobot_proxy;

	string proxy, tmp;
	initialize();


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "AGMExecutiveProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMExecutiveProxy\n";
		}
		agmexecutive_proxy = AGMExecutivePrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy AGMExecutive: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("AGMExecutiveProxy initialized Ok!");

	mprx["AGMExecutiveProxy"] = (::IceProxy::Ice::Object*)(&agmexecutive_proxy);//Remote server proxy creation example

	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "OmniRobotProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy OmniRobotProxy\n";
		}
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception creating proxy OmniRobot: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");

	mprx["OmniRobotProxy"] = (::IceProxy::Ice::Object*)(&omnirobot_proxy);//Remote server proxy creation example
	IceStorm::TopicManagerPrx topicManager;
	try
	{
		topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;
		return EXIT_FAILURE;
	}
	IceStorm::TopicPrx logger_topic;

	while (!logger_topic)
	{
		try
		{
			logger_topic = topicManager->retrieve("Logger");
		}
		catch (const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: ERROR retrieving Logger topic. \n";
			try
			{
				logger_topic = topicManager->create("Logger");
			}
			catch (const IceStorm::TopicExists&){
				// Another client created the topic.
				cout << "[" << PROGRAM_NAME << "]: ERROR publishing the Logger topic. It's possible that other component have created\n";
			}
		}
	}

	Ice::ObjectPrx logger_pub = logger_topic->getPublisher()->ice_oneway();
	logger_pubproxy = LoggerPrx::uncheckedCast(logger_pub);
	mprx["LoggerPub"] = (::IceProxy::Ice::Object*)(&logger_pubproxy);

	SpecificWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		try {
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, "")) {
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
			}
			Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
			CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor);
			adapterCommonBehavior->add(commonbehaviorI, Ice::stringToIdentity("commonbehavior"));
			adapterCommonBehavior->activate();
		}
		catch(const Ice::Exception& ex)
		{
			status = EXIT_FAILURE;

			cout << "[" << PROGRAM_NAME << "]: Exception raised while creating CommonBehavior adapter: " << endl;
			cout << ex;

		}



		try
		{
			// Server adapter creation and publication
			if (not GenericMonitor::configGetString(communicator(), prefix, "AGMCommonBehavior.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMCommonBehavior";
			}
			Ice::ObjectAdapterPtr adapterAGMCommonBehavior = communicator()->createObjectAdapterWithEndpoints("AGMCommonBehavior", tmp);
			AGMCommonBehaviorI *agmcommonbehavior = new AGMCommonBehaviorI(worker);
			adapterAGMCommonBehavior->add(agmcommonbehavior, Ice::stringToIdentity("agmcommonbehavior"));
			adapterAGMCommonBehavior->activate();
			cout << "[" << PROGRAM_NAME << "]: AGMCommonBehavior adapter created in port " << tmp << endl;
			}
			catch (const IceStorm::TopicExists&){
				cout << "[" << PROGRAM_NAME << "]: ERROR creating or activating adapter for AGMCommonBehavior\n";
			}



		// Server adapter creation and publication
		IceStorm::TopicPrx agmexecutivetopic_topic;
		Ice::ObjectPrx agmexecutivetopic;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "AGMExecutiveTopicTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AGMExecutiveTopicProxy";
			}
			Ice::ObjectAdapterPtr AGMExecutiveTopic_adapter = communicator()->createObjectAdapterWithEndpoints("agmexecutivetopic", tmp);
			AGMExecutiveTopicPtr agmexecutivetopicI_ =  new AGMExecutiveTopicI(worker);
			Ice::ObjectPrx agmexecutivetopic = AGMExecutiveTopic_adapter->addWithUUID(agmexecutivetopicI_)->ice_oneway();
			if(!agmexecutivetopic_topic)
			{
				try {
					agmexecutivetopic_topic = topicManager->create("AGMExecutiveTopic");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						agmexecutivetopic_topic = topicManager->retrieve("AGMExecutiveTopic");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				agmexecutivetopic_topic->subscribeAndGetPublisher(qos, agmexecutivetopic);
			}
			AGMExecutiveTopic_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating AGMExecutiveTopic topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		IceStorm::TopicPrx aprilbasedlocalization_topic;
		Ice::ObjectPrx aprilbasedlocalization;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "AprilBasedLocalizationTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilBasedLocalizationProxy";
			}
			Ice::ObjectAdapterPtr AprilBasedLocalization_adapter = communicator()->createObjectAdapterWithEndpoints("aprilbasedlocalization", tmp);
			AprilBasedLocalizationPtr aprilbasedlocalizationI_ =  new AprilBasedLocalizationI(worker);
			Ice::ObjectPrx aprilbasedlocalization = AprilBasedLocalization_adapter->addWithUUID(aprilbasedlocalizationI_)->ice_oneway();
			if(!aprilbasedlocalization_topic)
			{
				try {
					aprilbasedlocalization_topic = topicManager->create("AprilBasedLocalization");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						aprilbasedlocalization_topic = topicManager->retrieve("AprilBasedLocalization");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				aprilbasedlocalization_topic->subscribeAndGetPublisher(qos, aprilbasedlocalization);
			}
			AprilBasedLocalization_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating AprilBasedLocalization topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		IceStorm::TopicPrx cgr_topic;
		Ice::ObjectPrx cgr;
		try
		{
			if (not GenericMonitor::configGetString(communicator(), prefix, "CGRTopic.Endpoints", tmp, ""))
			{
				cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CGRProxy";
			}
			Ice::ObjectAdapterPtr CGR_adapter = communicator()->createObjectAdapterWithEndpoints("cgr", tmp);
			CGRPtr cgrI_ =  new CGRI(worker);
			Ice::ObjectPrx cgr = CGR_adapter->addWithUUID(cgrI_)->ice_oneway();
			if(!cgr_topic)
			{
				try {
					cgr_topic = topicManager->create("CGR");
				}
				catch (const IceStorm::TopicExists&) {
					//Another client created the topic
					try{
						cout << "[" << PROGRAM_NAME << "]: Probably other client already opened the topic. Trying to connect.\n";
						cgr_topic = topicManager->retrieve("CGR");
					}
					catch(const IceStorm::NoSuchTopic&)
					{
						cout << "[" << PROGRAM_NAME << "]: Topic doesn't exists and couldn't be created.\n";
						//Error. Topic does not exist
					}
				}
				IceStorm::QoS qos;
				cgr_topic->subscribeAndGetPublisher(qos, cgr);
			}
			CGR_adapter->activate();
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			cout << "[" << PROGRAM_NAME << "]: Error creating CGR topic.\n";
			//Error. Topic does not exist
		}

		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

		#ifdef USE_QTGUI
			//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
			a.setQuitOnLastWindowClosed( true );
		#endif
		// Run QT Application Event Loop
		a.exec();

		try
		{
			std::cout << "Unsubscribing topic: agmexecutivetopic " <<std::endl;
			agmexecutivetopic_topic->unsubscribe( agmexecutivetopic );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: agmexecutivetopic " <<std::endl;
		}
		try
		{
			std::cout << "Unsubscribing topic: aprilbasedlocalization " <<std::endl;
			aprilbasedlocalization_topic->unsubscribe( aprilbasedlocalization );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: aprilbasedlocalization " <<std::endl;
		}
		try
		{
			std::cout << "Unsubscribing topic: cgr " <<std::endl;
			cgr_topic->unsubscribe( cgr );
		}
		catch(const Ice::Exception& ex)
		{
			std::cout << "ERROR Unsubscribing topic: cgr " <<std::endl;
		}

		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

	}
	#ifdef USE_QTGUI
		a.quit();
	#endif

	status = EXIT_SUCCESS;
	monitor->terminate();
	monitor->wait();
	delete worker;
	delete monitor;
	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
	::localizationAgent app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
