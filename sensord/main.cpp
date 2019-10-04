/**
   @file main.cpp
   @brief Sensord initiation point

   <p>
   Copyright (C) 2009-2010 Nokia Corporation

   @author Semi Malinen <semi.malinen@nokia.com>
   @author Joep van Gassel <joep.van.gassel@nokia.com>
   @author Timo Rongas <ext-timo.2.rongas@nokia.com>
   @author Ustun Ergenoglu <ext-ustun.ergenoglu@nokia.com>
   @author Lihan Guo <ext-lihan.guo@nokia.com>
   @author Antti Virtanen <antti.i.virtanen@nokia.com

   This file is part of Sensord.

   Sensord is free software; you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License
   version 2.1 as published by the Free Software Foundation.

   Sensord is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with Sensord.  If not, see <http://www.gnu.org/licenses/>.
   </p>
 */

#include <QtCore/QCoreApplication>
#include <QtCore/QDebug>
#include <QSocketNotifier>

#include <systemd/sd-daemon.h>

#include <deviceinfo.h>

#include <unistd.h>
#include <signal.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>

#include "config.h"
#include "sensormanager.h"
#include "sensormanager_a.h"
#include "logging.h"
#include "calibrationhandler.h"
#include "parser.h"

static QtMsgType logLevel;
static QtMessageHandler previousMessageHandler;

static int normalizeLevel(QtMsgType type)
{
    /* Map QtMsgType enum values to something that hopefully
     * makes sense in less-than / greater-than sense too. */
    switch (type) {
    case QtDebugMsg:
        return 0;
    case QtInfoMsg:
        return 1;
    case QtWarningMsg:
        return 3;
    case QtCriticalMsg:
        return 4;
    default:
        return static_cast<int>(type);
    }
}

static void messageOutput(QtMsgType type, const QMessageLogContext &context, const QString &str)
{
    if (normalizeLevel(type) < normalizeLevel(logLevel))
        return;

    previousMessageHandler(type, context, str);
}

void printUsage();

void signalUSR1(int param)
{
    Q_UNUSED(param);
    if (logLevel != QtDebugMsg) {
        logLevel = QtDebugMsg;
        sensordLogW() << "Debug logging enabled";
    }
    else {
        logLevel = QtWarningMsg;
        sensordLogW() << "Debug logging disabled";
    }
}

void signalUSR2(int param)
{
    Q_UNUSED(param);

    QStringList output;

    output.append("Flushing sensord state");
    output.append(QString("  Logging level: %1").arg(logLevel));
    SensorManager::instance().printStatus(output);

    foreach (const QString& line, output) {
        sensordLogW() << line.toLocal8Bit().data();
    }
}

void signalINT(int param)
{
    signal(param, SIG_DFL);
    sensordLogD() << "Terminating ...";
    QCoreApplication::exit(0);
}

class SignalNotifier : public QObject
{
public:
     SignalNotifier();
     ~SignalNotifier();
private slots:
     void handleSignalInput(int socket);
private:
     static void handleAsyncSignal(int sig);
     QSocketNotifier *m_socketNotifier;
     static int s_pipe[2];
     static const int s_signals[];
};

SignalNotifier::SignalNotifier()
    : m_socketNotifier(0)
{
    sensordLogD() << "Setup async signal handlers";
    if (pipe2(s_pipe, O_CLOEXEC) == -1) {
        qFatal("Failed to create a pipe for signal passunc");
    }
    m_socketNotifier = new QSocketNotifier(s_pipe[0], QSocketNotifier::Read, this);
    connect(m_socketNotifier, &QSocketNotifier::activated,
            this, &SignalNotifier::handleSignalInput);
    struct sigaction action;
    memset(&action, 0, sizeof action);
    action.sa_handler = handleAsyncSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;
    for (size_t i = 0; s_signals[i] != -1; ++i )
        sigaction(s_signals[i], &action, 0);
}

SignalNotifier::~SignalNotifier()
{
    sensordLogD() << "Reset async signal handlers";
    for (size_t i = 0; s_signals[i] != -1; ++i )
        signal(s_signals[i], SIG_DFL);
    delete m_socketNotifier; m_socketNotifier = 0;
    close(s_pipe[1]), s_pipe[1] = -1;
    close(s_pipe[0]), s_pipe[0] = -1;
}

void SignalNotifier::handleAsyncSignal(int sig)
{
    /* Can call only async-signal safe functions! */
    if (write(s_pipe[1], &sig, sizeof sig) == -1) {
        _exit(EXIT_FAILURE);
    }
}

void SignalNotifier::handleSignalInput(int socket)
{
    Q_UNUSED(socket);
    int sig = SIGTERM;
    uint64_t tmp = 0;
    if (read(s_pipe[0], &sig, sizeof sig) == -1) {
        // dontcare
    }
    sensordLogD() << "Caught async signal"  << strsignal(sig);
    switch (sig) {
    case SIGINT:
    case SIGTERM:
        signalINT(sig);
        break;
    case SIGUSR1:
        signalUSR1(sig);
        break;
    case SIGUSR2:
        signalUSR2(sig);
        break;
    };
}

int SignalNotifier::s_pipe[2] = { -1, -1 };

const int SignalNotifier::s_signals[] =
{
    SIGINT, SIGTERM, SIGUSR1, SIGUSR2, -1
};

int main(int argc, char *argv[])
{
    previousMessageHandler = qInstallMessageHandler(messageOutput);

    QCoreApplication app(argc, argv);
    Parser parser(app.arguments());

    if (parser.printHelp())
    {
        printUsage();
        app.exit(EXIT_SUCCESS);
        return 0;

    }

    logLevel = parser.getLogLevel();

    const char* CONFIG_FILE_PATH = "/etc/sensorfw/sensord.conf";
    const char* CONFIG_DIR_PATH = "/etc/sensorfw/sensord.conf.d/";

    QString defConfigFile = CONFIG_FILE_PATH;
    if(parser.configFileInput())
    {
        defConfigFile = parser.configFilePath();
    }

    QString defConfigDir = CONFIG_DIR_PATH;
    if(parser.configDirInput())
    {
        defConfigDir = parser.configDirPath();
    }

    if (parser.deviceInfo())
    {
        DeviceInfo *deviceInfo = new DeviceInfo();
        if (deviceInfo->contains("sensorfwConfig")) {
            defConfigFile = QString::fromStdString(deviceInfo->get("sensorfwConfig",
                                                   defConfigFile.toStdString()));
        }
    }

    if (!SensorFrameworkConfig::loadConfig(defConfigFile, defConfigDir))
    {
        sensordLogC() << "SensorFrameworkConfig file error! Load using default paths.";
        if (!SensorFrameworkConfig::loadConfig(CONFIG_FILE_PATH, CONFIG_DIR_PATH))
        {
            sensordLogC() << "Which also failed. Bailing out";
            return 1;
        }
    }

    if (parser.createDaemon())
    {
        fflush(0);

        int pid = fork();

        if(pid < 0)
        {
            sensordLogC() << "Failed to create a daemon: " << strerror(errno);
            exit(EXIT_FAILURE);
        } else if (pid > 0) {
            sensordLogD() << "Created a daemon";
            _exit(EXIT_SUCCESS);
        }
    }

    SensorManager& sm = SensorManager::instance();

#ifdef PROVIDE_CONTEXT_INFO
    if (parser.contextInfo())
    {
        sensordLogD() << "Loading ContextSensor " << sm.loadPlugin("contextsensor");
        sensordLogD() << "Loading ALSSensor " << sm.loadPlugin("alssensor");
    }
#endif

    if (parser.magnetometerCalibration())
    {
        CalibrationHandler* calibrationHandler_ = new CalibrationHandler(NULL);
        calibrationHandler_->initiateSession();
        QObject::connect(&sm, SIGNAL(resumeCalibration()), calibrationHandler_, SLOT(resumeCalibration()));
        QObject::connect(&sm, SIGNAL(stopCalibration()), calibrationHandler_, SLOT(stopCalibration()));
    }

    if (!sm.registerService())
    {
        sensordLogW() << "Failed to register service on D-Bus. Aborting.";
        exit(EXIT_FAILURE);
    }

    if (parser.notifySystemd())
    {
        sd_notify(0, "READY=1");
    }

    SignalNotifier *signalNotifier = new SignalNotifier();
    int ret = app.exec();
    delete signalNotifier; signalNotifier = 0;

    sensordLogD() << "Exiting...";
    SensorFrameworkConfig::close();
    return ret;
}

void printUsage()
{
    qDebug() << "Usage: sensord [OPTIONS]";
    qDebug() << " -d, --daemon                     Detach from terminal and run as daemon.\n";
    qDebug() << " -s, --systemd                    Notify systemd when ready.\n";
    qDebug() << " -l=N, --log-level=<level>        Use given logging level. Messages are logged for";
    qDebug() << "                                  the given and higher priority levels. Level";
    qDebug() << "                                  can also be notched up by sending SIGUSR1 to";
    qDebug() << "                                  the process. Valid values for N are: 'test',";
    qDebug() << "                                  'debug', 'warning', 'critical'.\n";
    qDebug() << " -c=P, --config-file=<path>       Load configuration from given path. By default";
    qDebug() << "                                  /etc/sensorfw/sensord.conf is used.\n";
    qDebug() << " --no-context-info                Do not provide context information for context";
    qDebug() << "                                  framework.\n";
    qDebug() << " --no-magnetometer-bg-calibration Do not start calibration of magnetometer in";
    qDebug() << "                                  the background.\n";
    qDebug() << " -h, --help                       Show usage info and exit.";
}
