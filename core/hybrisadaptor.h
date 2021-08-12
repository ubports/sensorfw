/****************************************************************************
**
** Copyright (C) 2013 Jolla Ltd
** Contact: lorn.potter@jollamobile.com
**
**
** $QT_BEGIN_LICENSE:LGPL$
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef HybrisAdaptor_H
#define HybrisAdaptor_H

#include <QObject>
#include <QThread>
#include <QTimer>
#include <QFile>

#include "deviceadaptor.h"

#ifdef USE_BINDER
#include <gbinder.h>
#include "hybrisbindertypes.h"

#include <sys/user.h>
#ifndef PAGE_SIZE
#define PAGE_SHIFT		12
#define PAGE_SIZE		(1UL << PAGE_SHIFT)
#endif
#include <memory>
#include <fmq/MessageQueue.h>
#include <fmq/EventFlag.h>

#include "android/hardware/sensors/1.0/ISensors.h"
#include "android/hardware/sensors/2.0/ISensors.h"
#else
#include <hardware/sensors.h>
#include <pthread.h>
#endif

#define SENSORFW_MCE_WATCHER

class HybrisAdaptor;

struct HybrisSensorState
{
    HybrisSensorState();
    ~HybrisSensorState();

    int  m_minDelay;
    int  m_maxDelay;
    int  m_delay;
    int  m_active;
    sensors_event_t m_fallbackEvent;
};

class HybrisManager : public QObject
{
    Q_OBJECT
public:
    static HybrisManager *instance();

    explicit HybrisManager(QObject *parent = 0);
    virtual ~HybrisManager();
    void cleanup();
    void initManager();

    /* - - - - - - - - - - - - - - - - - - - *
     * android sensor functions
     * - - - - - - - - - - - - - - - - - - - */

    sensors_event_t *eventForHandle(int handle) const;
    int              indexForHandle(int handle) const;
    int              indexForType  (int sensorType) const;
    int              handleForType (int sensorType) const;
    float            getMaxRange   (int handle) const;
    float            getResolution (int handle) const;
    int              getMinDelay   (int handle) const;
    int              getMaxDelay   (int handle) const;
    int              getDelay      (int handle) const;
    bool             setDelay      (int handle, int delay_ms, bool force);
    bool             getActive     (int handle) const;
    bool             setActive     (int handle, bool active);

    /* - - - - - - - - - - - - - - - - - - - *
     * HybrisManager <--> sensorfwd
     * - - - - - - - - - - - - - - - - - - - */

    void startReader     (HybrisAdaptor *adaptor);
    void stopReader      (HybrisAdaptor *adaptor);
    void registerAdaptor (HybrisAdaptor * adaptor);
    void processSample   (const sensors_event_t& data);

private:
    // fields
    bool                          m_initialized;
    QMap <int, HybrisAdaptor *>   m_registeredAdaptors; // type -> obj

#ifdef USE_BINDER
    // Binder backend
    GBinderClient                *m_client;
    gulong                        m_deathId;
    gulong                        m_pollTransactId;
    GBinderRemoteObject          *m_remote;
    GBinderServiceManager        *m_serviceManager;
    struct sensor_t              *m_sensorArray;   // [m_sensorCount]

    typedef android::hardware::MessageQueue<android::hardware::sensors::V1_0::Event, android::hardware::kSynchronizedReadWrite> EventMessageQueue;
    typedef android::hardware::MessageQueue<uint32_t, android::hardware::kSynchronizedReadWrite> WakeLockQueue;
    std::unique_ptr<EventMessageQueue> m_eventQueue;
    std::unique_ptr<WakeLockQueue> m_wakeLockQueue;
    android::hardware::EventFlag* m_eventQueueFlag;
    android::hardware::EventFlag* m_wakeLockQueueFlag;
    android::sp<android::hardware::sensors::V2_0::ISensors> m_sensors;
    pthread_t                     m_fmqEventReaderTid;
#else
    // HAL backend
    struct sensors_module_t      *m_halModule;
    struct sensors_poll_device_t *m_halDevice;
    pthread_t                     m_halEventReaderTid;
    const struct sensor_t        *m_sensorArray;   // [m_sensorCount]
#endif
    int                           m_sensorCount;
    HybrisSensorState            *m_sensorState;   // [m_sensorCount]
    QMap <int, int>               m_indexOfType;   // type   -> index
    QMap <int, int>               m_indexOfHandle; // handle -> index

#ifdef USE_BINDER
    void getSensorList();
    void getSensorList_2_0();
    void startConnect();
    void finishConnect();
    void finishConnect_2_0();
    static void binderDied(GBinderRemoteObject *, void *user_data);
    void pollEvents();
    static void pollEventsCallback(
        GBinderClient* /*client*/, GBinderRemoteReply* reply,
        int status, void* userData);
#endif

    friend class HybrisAdaptorReader;

#ifdef USE_BINDER
    static void *fmqEventReaderThread(void *aptr);
#else
private:
    static void *halEventReaderThread(void *aptr);
#endif
    void processEvents(const sensors_event_t *buffer,
        int numberOfEvents, bool &blockSuspend, bool &errorInInput);
};

class HybrisAdaptor : public DeviceAdaptor
{
public:
    HybrisAdaptor(const QString& id, int type);
    virtual ~HybrisAdaptor();

    virtual void init();

    virtual bool startAdaptor();
    bool         isRunning() const;
    virtual void stopAdaptor();

    void         evaluateSensor();
    virtual bool startSensor();
    virtual void stopSensor();

    virtual bool standby();
    virtual bool resume();

    virtual void sendInitialData();

    friend class HybrisManager;

protected:
    virtual void processSample(const sensors_event_t& data) = 0;

    qreal        minRange() const;
    qreal        maxRange() const;
    qreal        resolution() const;

    unsigned int minInterval() const;
    unsigned int maxInterval() const;

    virtual unsigned int interval() const;
    virtual bool setInterval(const unsigned int value, const int sessionId);
    virtual unsigned int evaluateIntervalRequests(int& sessionId) const;
    static bool writeToFile(const QByteArray& path, const QByteArray& content);

private:
    bool          m_inStandbyMode;
    volatile bool m_isRunning;
    bool          m_shouldBeRunning;

    int           m_sensorHandle;
    int           m_sensorType;
};

#endif // HybrisAdaptor_H
