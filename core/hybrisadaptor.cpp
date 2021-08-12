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

#include "hybrisadaptor.h"
#include "deviceadaptor.h"

#include <QDebug>
#include <QCoreApplication>
#include <QTimer>

#ifdef USE_BINDER
#include "android/hardware/sensors/2.0/ISensorsCallback.h"
#include "android/hardware/sensors/2.0/types.h"
#else
#include <hardware/hardware.h>
#endif

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <signal.h>


/* Older devices probably have old android hal and thus do
 * not define sensor all sensor types that have been added
 * later on -> In order to both use symbolic names and
 * compile for all devices we need to fill in holes that
 * android hal for some particular device might have.
 */
#ifndef SENSOR_TYPE_META_DATA
#define SENSOR_TYPE_META_DATA                        (0)
#endif
#ifndef SENSOR_TYPE_ACCELEROMETER
#define SENSOR_TYPE_ACCELEROMETER                    (1)
#endif
#ifndef SENSOR_TYPE_GEOMAGNETIC_FIELD
#define SENSOR_TYPE_GEOMAGNETIC_FIELD                (2) // alias for SENSOR_TYPE_MAGNETIC_FIELD
#endif
#ifndef SENSOR_TYPE_MAGNETIC_FIELD
#define SENSOR_TYPE_MAGNETIC_FIELD                   (2) // alias for SENSOR_TYPE_GEOMAGNETIC_FIELD
#endif
#ifndef SENSOR_TYPE_ORIENTATION
#define SENSOR_TYPE_ORIENTATION                      (3)
#endif
#ifndef SENSOR_TYPE_GYROSCOPE
#define SENSOR_TYPE_GYROSCOPE                        (4)
#endif
#ifndef SENSOR_TYPE_LIGHT
#define SENSOR_TYPE_LIGHT                            (5)
#endif
#ifndef SENSOR_TYPE_PRESSURE
#define SENSOR_TYPE_PRESSURE                         (6)
#endif
#ifndef SENSOR_TYPE_TEMPERATURE
#define SENSOR_TYPE_TEMPERATURE                      (7)
#endif
#ifndef SENSOR_TYPE_PROXIMITY
#define SENSOR_TYPE_PROXIMITY                        (8)
#endif
#ifndef SENSOR_TYPE_GRAVITY
#define SENSOR_TYPE_GRAVITY                          (9)
#endif
#ifndef SENSOR_TYPE_LINEAR_ACCELERATION
#define SENSOR_TYPE_LINEAR_ACCELERATION             (10)
#endif
#ifndef SENSOR_TYPE_ROTATION_VECTOR
#define SENSOR_TYPE_ROTATION_VECTOR                 (11)
#endif
#ifndef SENSOR_TYPE_RELATIVE_HUMIDITY
#define SENSOR_TYPE_RELATIVE_HUMIDITY               (12)
#endif
#ifndef SENSOR_TYPE_AMBIENT_TEMPERATURE
#define SENSOR_TYPE_AMBIENT_TEMPERATURE             (13)
#endif
#ifndef SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED
#define SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED     (14)
#endif
#ifndef SENSOR_TYPE_GAME_ROTATION_VECTOR
#define SENSOR_TYPE_GAME_ROTATION_VECTOR            (15)
#endif
#ifndef SENSOR_TYPE_GYROSCOPE_UNCALIBRATED
#define SENSOR_TYPE_GYROSCOPE_UNCALIBRATED          (16)
#endif
#ifndef SENSOR_TYPE_SIGNIFICANT_MOTION
#define SENSOR_TYPE_SIGNIFICANT_MOTION              (17)
#endif
#ifndef SENSOR_TYPE_STEP_DETECTOR
#define SENSOR_TYPE_STEP_DETECTOR                   (18)
#endif
#ifndef SENSOR_TYPE_STEP_COUNTER
#define SENSOR_TYPE_STEP_COUNTER                    (19)
#endif
#ifndef SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR
#define SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR     (20)
#endif
#ifndef SENSOR_TYPE_HEART_RATE
#define SENSOR_TYPE_HEART_RATE                      (21)
#endif
#ifndef SENSOR_TYPE_TILT_DETECTOR
#define SENSOR_TYPE_TILT_DETECTOR                   (22)
#endif
#ifndef SENSOR_TYPE_WAKE_GESTURE
#define SENSOR_TYPE_WAKE_GESTURE                    (23)
#endif
#ifndef SENSOR_TYPE_GLANCE_GESTURE
#define SENSOR_TYPE_GLANCE_GESTURE                  (24)
#endif
#ifndef SENSOR_TYPE_PICK_UP_GESTURE
#define SENSOR_TYPE_PICK_UP_GESTURE                 (25)
#endif
#ifndef SENSOR_TYPE_WRIST_TILT_GESTURE
#define SENSOR_TYPE_WRIST_TILT_GESTURE              (26)
#endif

#ifdef USE_BINDER
#define SENSOR_BINDER_SERVICE_DEVICE "/dev/hwbinder"
#define SENSOR_BINDER_SERVICE_IFACE "android.hardware.sensors@1.0::ISensors"
#define SENSOR_BINDER_SERVICE_NAME  SENSOR_BINDER_SERVICE_IFACE "/default"
#define SENSOR_BINDER_SERVICE_IFACE_2_0 "android.hardware.sensors@2.0::ISensors"
#define SENSOR_BINDER_SERVICE_NAME_2_0  SENSOR_BINDER_SERVICE_IFACE_2_0 "/default"
#define MAX_RECEIVE_BUFFER_EVENT_COUNT 128
#endif

/* ========================================================================= *
 * UTILITIES
 * ========================================================================= */

static char const *
sensorTypeName(int type)
{
    switch (type) {
    case SENSOR_TYPE_META_DATA:                   return "META_DATA";
    case SENSOR_TYPE_ACCELEROMETER:               return "ACCELEROMETER";
    case SENSOR_TYPE_GEOMAGNETIC_FIELD:           return "GEOMAGNETIC_FIELD";
    case SENSOR_TYPE_ORIENTATION:                 return "ORIENTATION";
    case SENSOR_TYPE_GYROSCOPE:                   return "GYROSCOPE";
    case SENSOR_TYPE_LIGHT:                       return "LIGHT";
    case SENSOR_TYPE_PRESSURE:                    return "PRESSURE";
    case SENSOR_TYPE_TEMPERATURE:                 return "TEMPERATURE";
    case SENSOR_TYPE_PROXIMITY:                   return "PROXIMITY";
    case SENSOR_TYPE_GRAVITY:                     return "GRAVITY";
    case SENSOR_TYPE_LINEAR_ACCELERATION:         return "LINEAR_ACCELERATION";
    case SENSOR_TYPE_ROTATION_VECTOR:             return "ROTATION_VECTOR";
    case SENSOR_TYPE_RELATIVE_HUMIDITY:           return "RELATIVE_HUMIDITY";
    case SENSOR_TYPE_AMBIENT_TEMPERATURE:         return "AMBIENT_TEMPERATURE";
    case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED: return "MAGNETIC_FIELD_UNCALIBRATED";
    case SENSOR_TYPE_GAME_ROTATION_VECTOR:        return "GAME_ROTATION_VECTOR";
    case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:      return "GYROSCOPE_UNCALIBRATED";
    case SENSOR_TYPE_SIGNIFICANT_MOTION:          return "SIGNIFICANT_MOTION";
    case SENSOR_TYPE_STEP_DETECTOR:               return "STEP_DETECTOR";
    case SENSOR_TYPE_STEP_COUNTER:                return "STEP_COUNTER";
    case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR: return "GEOMAGNETIC_ROTATION_VECTOR";
    case SENSOR_TYPE_HEART_RATE:                  return "HEART_RATE";
    case SENSOR_TYPE_TILT_DETECTOR:               return "TILT_DETECTOR";
    case SENSOR_TYPE_WAKE_GESTURE:                return "WAKE_GESTURE";
    case SENSOR_TYPE_GLANCE_GESTURE:              return "GLANCE_GESTURE";
    case SENSOR_TYPE_PICK_UP_GESTURE:             return "PICK_UP_GESTURE";
    case SENSOR_TYPE_WRIST_TILT_GESTURE:          return "WRIST_TILT_GESTURE";
    }

    static char buf[32];
    snprintf(buf, sizeof buf, "type%d", type);
    return buf;
}

static void ObtainTemporaryWakeLock()
{
    static bool triedToOpen = false;
    static int wakeLockFd = -1;

    if (!triedToOpen) {
        triedToOpen = true;
        wakeLockFd = ::open("/sys/power/wake_lock", O_RDWR);
        if (wakeLockFd == -1) {
            sensordLogW() << "wake locks not available:" << ::strerror(errno);
        }
    }

    if (wakeLockFd != -1) {
        sensordLogD() << "wake lock to guard sensor data io";
        static const char m[] = "sensorfwd_pass_data 1000000000\n";
        if (::write(wakeLockFd, m, sizeof m - 1) == -1) {
            sensordLogW() << "wake locking failed:" << ::strerror(errno);
            ::close(wakeLockFd), wakeLockFd = -1;
        }
    }
}

/* ========================================================================= *
 * HybrisSensorState
 * ========================================================================= */

HybrisSensorState::HybrisSensorState()
    : m_minDelay(0)
    , m_maxDelay(0)
    , m_delay(-1)
    , m_active(-1)
{
    memset(&m_fallbackEvent, 0, sizeof m_fallbackEvent);
}

HybrisSensorState::~HybrisSensorState()
{
}

/* ========================================================================= *
 * HybrisManager
 * ========================================================================= */

Q_GLOBAL_STATIC(HybrisManager, hybrisManager)

HybrisManager::HybrisManager(QObject *parent)
    : QObject(parent)
    , m_initialized(false)
    , m_registeredAdaptors()
#ifdef USE_BINDER
    , m_client(NULL)
    , m_deathId(0)
    , m_pollTransactId(0)
    , m_remote(NULL)
    , m_serviceManager(NULL)
#else
    , m_halModule(NULL)
    , m_halDevice(NULL)
    , m_halEventReaderTid(0)
#endif
    , m_sensorArray(NULL)
    , m_sensorCount(0)
    , m_sensorState(NULL)
    , m_indexOfType()
    , m_indexOfHandle()
{
#ifdef USE_BINDER
    startConnect();
#else
    int err;

    /* Open android sensor plugin */
    err = hw_get_module(SENSORS_HARDWARE_MODULE_ID,
                        (hw_module_t const**)&m_halModule);
    if (err != 0) {
        m_halModule = 0;
        sensordLogW() << "hw_get_module() failed" <<  strerror(-err);
        return ;
    }

    /* Open android sensor device */
    err = sensors_open(&m_halModule->common, &m_halDevice);
    if (err != 0) {
        m_halDevice = 0;
        sensordLogW() << "sensors_open() failed:" << strerror(-err);
        return;
    }

    /* Get static sensor information */
    m_sensorCount = m_halModule->get_sensors_list(m_halModule, &m_sensorArray);

    initManager();
#endif
}

void HybrisManager::initManager()
{
    /* Reserve space for sensor state data */
    m_sensorState = new HybrisSensorState[m_sensorCount];

    /* Select and initialize sensors to be used */
    for (int i = 0 ; i < m_sensorCount ; i++) {
        /* Always do handle -> index mapping */
        m_indexOfHandle.insert(m_sensorArray[i].handle, i);

        bool use = true;
        // Assumption: The primary sensor variants that we want to
        // use are listed before the secondary ones that we want
        // to ignore -> Use the 1st entry found for each sensor type.
        if (m_indexOfType.contains(m_sensorArray[i].type)) {
            use = false;
        }

        // some devices have compass and compass raw,
        // ignore compass raw. compass has range 360
        if (m_sensorArray[i].type == SENSOR_TYPE_ORIENTATION &&
            m_sensorArray[i].maxRange != 360) {
            use = false;
        }

        sensordLogD() << Q_FUNC_INFO
            << (use ? "SELECT" : "IGNORE")
            << "type:" << m_sensorArray[i].type
#ifdef USE_BINDER
            << "name:" << (m_sensorArray[i].name.data.str ?: "n/a");
#else
            << "name:" << (m_sensorArray[i].name ?: "n/a");
#endif

        if (use) {
            // min/max delay is specified in [us] -> convert to [ms]
            int minDelay = (m_sensorArray[i].minDelay + 999) / 1000;
            int maxDelay = -1; // Assume: not defined by hal

#ifdef USE_BINDER
            maxDelay = (m_sensorArray[i].maxDelay + 999) / 1000;
#else
#ifdef SENSORS_DEVICE_API_VERSION_1_3
            if (m_halDevice->common.version >= SENSORS_DEVICE_API_VERSION_1_3)
                maxDelay = (m_sensorArray[i].maxDelay + 999) / 1000;
#endif
#endif

            /* If HAL does not define maximum delay, we need to invent
             * something that a) allows sensorfwd logic to see a range
             * instead of a point, b) is unlikely to be wrong enough to
             * cause problems...
             *
             * For now use: minDelay * 2, but at least 1000 ms.
             */

            if (maxDelay < 0 && minDelay > 0) {
                maxDelay = (minDelay < 500) ? 1000 : (minDelay * 2);
                sensordLogD("hal does not specify maxDelay, fallback: %d ms",
                            maxDelay);
            }

            // Positive minDelay means delay /can/ be set - but depending
            // on sensor hal implementation it can also mean that some
            // delay /must/ be set or the sensor does not start reporting
            // despite being enabled -> as an protection agains clients
            // failing to explicitly set delays / using delays that would
            // get rejected by upper levels of sensorfwd logic -> setup
            // 200 ms delay (capped to reported min/max range).
            if (minDelay >= 0) {
                if (maxDelay < minDelay)
                    maxDelay = minDelay;

                int delay = minDelay ? 200 : 0;
                if (delay < minDelay)
                    delay = minDelay;
                else if (delay > maxDelay )
                    delay = maxDelay;

                m_sensorState[i].m_minDelay = minDelay;
                m_sensorState[i].m_maxDelay = maxDelay;

                setDelay(m_sensorArray[i].handle, delay, true);

                sensordLogD("delay = %d [%d, %d]",
                            m_sensorState[i].m_delay,
                            m_sensorState[i].m_minDelay,
                            m_sensorState[i].m_maxDelay);
            }
            m_indexOfType.insert(m_sensorArray[i].type, i);

            /* Set sane fallback values for select sensors in case the
             * hal does not report initial values. */

            sensors_event_t *eve = &m_sensorState[i].m_fallbackEvent;
#ifndef USE_BINDER
            eve->version = sizeof *eve;
#endif
            eve->sensor  = m_sensorArray[i].handle;
            eve->type    = m_sensorArray[i].type;
            switch (m_sensorArray[i].type) {
            case SENSOR_TYPE_LIGHT:
                // Roughly indoor lightning
#ifdef USE_BINDER
                eve->u.scalar = 400;
#else
                eve->light = 400;
#endif
                break;

            case SENSOR_TYPE_PROXIMITY:
                // Not-covered
#ifdef USE_BINDER
                eve->u.scalar = m_sensorArray[i].maxRange;
#else
                eve->distance = m_sensorArray[i].maxRange;
#endif
                break;
            default:
                eve->sensor  = 0;
                eve->type    = 0;
                break;
            }
        }

        /* Make sure all sensors are initially in stopped state */
        setActive(m_sensorArray[i].handle, false);
    }

#ifdef USE_BINDER
    if (m_sensors) {
        int err;
        /* Start android sensor event reader */
        err = pthread_create(&m_fmqEventReaderTid, 0, fmqEventReaderThread, this);
        if (err) {
            m_fmqEventReaderTid = 0;
            sensordLogC() << "Failed to start FMQ reader thread";
            return;
        }
        sensordLogD() << "FMQ reader thread started";
    } else {
        pollEvents();
    }
#else
    int err;
    /* Start android sensor event reader */
    err = pthread_create(&m_halEventReaderTid, 0, halEventReaderThread, this);
    if (err) {
        m_halEventReaderTid = 0;
        sensordLogC() << "Failed to start hal reader thread";
        return;
    }
    sensordLogD() << "Hal reader thread started";

    m_initialized = true;
#endif
}

HybrisManager::~HybrisManager()
{
    cleanup();

#ifdef USE_BINDER
    if (m_serviceManager) {
        gbinder_servicemanager_unref(m_serviceManager);
        m_serviceManager = NULL;
    }
#endif
}

void HybrisManager::cleanup()
{
    sensordLogD() << "stop all sensors";
    foreach (HybrisAdaptor *adaptor, m_registeredAdaptors.values()) {
        adaptor->stopSensor();
    }
#ifdef USE_BINDER
    if (m_pollTransactId) {
        gbinder_client_cancel(m_client, m_pollTransactId);
        m_pollTransactId = 0;
    }

    if (m_client) {
        gbinder_client_unref(m_client);
        m_client = NULL;
    }

    if (m_remote) {
        if (m_deathId) {
            gbinder_remote_object_remove_handler(m_remote, m_deathId);
            m_deathId = 0;
        }
        gbinder_remote_object_unref(m_remote);
        m_remote = NULL;
    }

    for (int i = 0 ; i < m_sensorCount ; i++) {
        g_free((void*)m_sensorArray[i].name.data.str);
        g_free((void*)m_sensorArray[i].vendor.data.str);
        g_free((void*)m_sensorArray[i].typeAsString.data.str);
        g_free((void*)m_sensorArray[i].requiredPermission.data.str);
    }
    delete[] m_sensorArray;
    m_sensorArray = NULL;
#else
    if (m_halDevice) {
        sensordLogD() << "close sensor device";
        int errorCode = sensors_close(m_halDevice);
        if (errorCode != 0) {
            sensordLogW() << "sensors_close() failed:" << strerror(-errorCode);
        }
        m_halDevice = NULL;
    }

    if (m_halEventReaderTid) {
        sensordLogD() << "Canceling hal reader thread";
        int err = pthread_cancel(m_halEventReaderTid);
        if (err) {
            sensordLogC() << "Failed to cancel hal reader thread";
        } else {
            sensordLogD() << "Waiting for hal reader thread to exit";
            void *ret = 0;
            struct timespec tmo = { 0, 0};
            clock_gettime(CLOCK_REALTIME, &tmo);
            tmo.tv_sec += 3;
            err = pthread_timedjoin_np(m_halEventReaderTid, &ret, &tmo);
            if (err) {
                sensordLogC() << "Hal reader thread did not exit";
            } else {
                sensordLogD() << "Hal reader thread terminated";
                m_halEventReaderTid = 0;
            }
        }
        if (m_halEventReaderTid) {
            /* The reader thread is stuck at android hal blob.
             * Continuing would be likely to release resourse
             * still in active use and lead to segfaulting.
             * Resort to doing a quick and dirty exit. */
            _exit(EXIT_FAILURE);
        }
    }
#endif
    delete[] m_sensorState;
    m_sensorState = NULL;
    m_sensorCount = 0;
    m_initialized = false;
}

HybrisManager *HybrisManager::instance()
{
    HybrisManager *priv = hybrisManager();
    return priv;
}

#ifdef USE_BINDER

void HybrisManager::getSensorList()
{
    sensordLogD() << "Get sensor list";
    GBinderReader reader;
    GBinderRemoteReply *reply;
    int status;

    reply = gbinder_client_transact_sync_reply(m_client, GET_SENSORS_LIST, NULL, &status);

    if (status != GBINDER_STATUS_OK) {
        sensordLogW() << "Unable to get sensor list: status " << status;
        cleanup();
        sleep(1);
        startConnect();
        return;
    }

    gbinder_remote_reply_init_reader(reply, &reader);
    gbinder_reader_read_int32(&reader, &status);
    gsize count = 0;
    gsize vecSize = 0;
    sensor_t *vec = (sensor_t *)gbinder_reader_read_hidl_vec(&reader, &count, &vecSize);

    m_sensorCount = count;
    m_sensorArray = new sensor_t[m_sensorCount];
    for (int i = 0 ; i < m_sensorCount ; i++) {
        memcpy(&m_sensorArray[i], &vec[i], sizeof(sensor_t));

        // Read strings
        GBinderBuffer *buffer = gbinder_reader_read_buffer(&reader);
        m_sensorArray[i].name.data.str = g_strdup((const gchar *)buffer->data);
        m_sensorArray[i].name.len = buffer->size;
        m_sensorArray[i].name.owns_buffer = true;
        gbinder_buffer_free(buffer);

        buffer = gbinder_reader_read_buffer(&reader);
        m_sensorArray[i].vendor.data.str = g_strdup((const gchar *)buffer->data);
        m_sensorArray[i].vendor.len = buffer->size;
        m_sensorArray[i].vendor.owns_buffer = true;
        gbinder_buffer_free(buffer);

        buffer = gbinder_reader_read_buffer(&reader);
        m_sensorArray[i].typeAsString.data.str = g_strdup((const gchar *)buffer->data);
        m_sensorArray[i].typeAsString.len = buffer->size;
        m_sensorArray[i].typeAsString.owns_buffer = true;
        gbinder_buffer_free(buffer);

        buffer = gbinder_reader_read_buffer(&reader);
        m_sensorArray[i].requiredPermission.data.str = g_strdup((const gchar *)buffer->data);
        m_sensorArray[i].requiredPermission.len = buffer->size;
        m_sensorArray[i].requiredPermission.owns_buffer = true;
        gbinder_buffer_free(buffer);
    }
    gbinder_remote_reply_unref(reply);

    initManager();

    m_initialized = true;
    sensordLogW() << "Hybris sensor manager initialized";
}

void HybrisManager::getSensorList_2_0()
{
    sensordLogD() << "Get sensor 2.0 list";

    auto ret = m_sensors->getSensorsList(
        [&](const auto &list) {
            m_sensorCount = list.size();
            m_sensorArray = new sensor_t[m_sensorCount];

            for (int i = 0; i < m_sensorCount; i++) {
                const auto& sensor = list[i];
                memcpy(&m_sensorArray[i], &sensor, sizeof(sensor_t));

                // Populate gbinder string structs
                m_sensorArray[i].name.data.str = g_strdup((const gchar *)sensor.name.c_str());
                m_sensorArray[i].name.len = sensor.name.size();
                m_sensorArray[i].name.owns_buffer = true;

                m_sensorArray[i].vendor.data.str = g_strdup((const gchar *)sensor.vendor.c_str());
                m_sensorArray[i].vendor.len = sensor.vendor.size();
                m_sensorArray[i].vendor.owns_buffer = true;

                m_sensorArray[i].typeAsString.data.str = g_strdup((const gchar *)sensor.typeAsString.c_str());
                m_sensorArray[i].typeAsString.len = sensor.typeAsString.size();
                m_sensorArray[i].typeAsString.owns_buffer = true;

                m_sensorArray[i].requiredPermission.data.str = g_strdup((const gchar *)sensor.requiredPermission.c_str());
                m_sensorArray[i].requiredPermission.len = sensor.requiredPermission.size();
                m_sensorArray[i].requiredPermission.owns_buffer = true;
            }
    });

    if (!ret.isOk()) {
        sensordLogW() << "Unable to get sensor 2.0 list: status " << ret.description().c_str();
        cleanup();
        sleep(1);
        startConnect();
        return;
    }

    initManager();

    m_initialized = true;
    sensordLogW() << "Hybris sensor manager initialized";
}

void HybrisManager::binderDied(GBinderRemoteObject *, void *user_data)
{
    HybrisManager *conn =
                    static_cast<HybrisManager *>(user_data);
    sensordLogW() << "Sensor service died! Trying to reconnect.";
    conn->cleanup();
    conn->startConnect();
}

void HybrisManager::startConnect()
{
    if (!m_serviceManager) {
        m_serviceManager = gbinder_servicemanager_new(SENSOR_BINDER_SERVICE_DEVICE);
    }

    if (gbinder_servicemanager_wait(m_serviceManager, -1)) {
        finishConnect();
    } else {
        sensordLogW() << "Could not get service manager for sensor service";
        cleanup();
    }
}

void HybrisManager::finishConnect()
{
    m_remote = gbinder_servicemanager_get_service_sync(m_serviceManager,
                                    SENSOR_BINDER_SERVICE_NAME, NULL);
    if (!m_remote) {
        sensordLogD() << "Could not find remote object for sensor service. Trying to connect to sensor 2.0 service.";
        finishConnect_2_0();
        return;
    } else {
        gbinder_remote_object_ref(m_remote);
        sensordLogD() << "Connected to sensor service";
        m_deathId = gbinder_remote_object_add_death_handler(m_remote, binderDied,
                        this);
        m_client = gbinder_client_new(m_remote, SENSOR_BINDER_SERVICE_IFACE);
        if (!m_client) {
            sensordLogD() << "Could not create client for sensor service. Trying to reconnect.";
        } else {
            // Sometimes sensor service has lingering connetion from
            // previous client which causes sensor service to restart
            // and we need to test with poll if remote is really working.
            GBinderRemoteReply *reply;
            GBinderLocalRequest *req = gbinder_client_new_request(m_client);
            int32_t status;

            // Empty poll to test if remote is working
            req = gbinder_local_request_append_int32(req, 0);

            reply = gbinder_client_transact_sync_reply(m_client, POLL, req, &status);
            gbinder_local_request_unref(req);
            gbinder_remote_reply_unref(reply);

            if (status != GBINDER_STATUS_OK) {
                sensordLogW() << "Poll failed with status" << status << ". Trying to reconnect.";
            } else {
                getSensorList();
                return;
            }
        }
    }
    // On failure cleanup and wait before reconnecting
    cleanup();
    sleep(1);
    startConnect();
}

void writeMQDescriptor(GBinderWriter* writer, const android::hardware::MQDescriptor<sensors_event_t, (android::hardware::MQFlavor)1u> &obj)
{
    using android::hardware::MQDescriptor;
    using android::hardware::GrantorDescriptor;

    GBinderParent parent;
    // write buffer
    parent.index = gbinder_writer_append_buffer_object(writer, &obj, sizeof(MQDescriptor<sensors_event_t, (android::hardware::MQFlavor)1u>));

    // write grantors hidl_vec data
    parent.offset = MQDescriptor<sensors_event_t, (android::hardware::MQFlavor)1u>::kOffsetOfGrantors;
    const GrantorDescriptor* data = obj.grantors().data();
    const gsize length = sizeof(GrantorDescriptor) * obj.grantors().size();
    gbinder_writer_append_buffer_object_with_parent(writer, data, length, &parent);

    // write native handle
    parent.offset = MQDescriptor<sensors_event_t, (android::hardware::MQFlavor)1u>::kOffsetOfHandle;
}

void HybrisManager::finishConnect_2_0()
{
    using namespace android::hardware::sensors;
    using android::hardware::EventFlag;

    m_sensors = V2_0::ISensors::getService();

    if (m_sensors == nullptr) {
        sensordLogD() << "Could not find remote object for sensor 2.0 service. Trying to reconnect.";
    } else {
        m_eventQueue = std::make_unique<EventMessageQueue>(
                MAX_RECEIVE_BUFFER_EVENT_COUNT,
                true /* configureEventFlagWord */);

        m_wakeLockQueue = std::make_unique<WakeLockQueue>(
                MAX_RECEIVE_BUFFER_EVENT_COUNT,
                true /* configureEventFlagWord */);

        EventFlag::deleteEventFlag(&m_eventQueueFlag);
        EventFlag::createEventFlag(m_eventQueue->getEventFlagWord(), &m_eventQueueFlag);

        EventFlag::deleteEventFlag(&m_wakeLockQueueFlag);
        EventFlag::createEventFlag(m_wakeLockQueue->getEventFlagWord(),
                                             &m_wakeLockQueueFlag);

        assert(m_sensors != nullptr && m_eventQueue != nullptr &&
                m_wakeLockQueue != nullptr && m_eventQueueFlag != nullptr &&
                m_wakeLockQueueFlag != nullptr);

        auto ret = m_sensors->initialize(
                *m_eventQueue->getDesc(),
                *m_wakeLockQueue->getDesc(),
                nullptr);

        if (!ret.isOk()) {
            sensordLogW() << "Initialize failed with status" << ret.description().c_str() << ". Trying to reconnect.";
            m_sensors = nullptr;
        } else {
            getSensorList_2_0();
            return;
        }
    }
    // On failure cleanup and wait before reconnecting
    cleanup();
    sleep(1);
    startConnect();
}
#endif //USE_BINDER

int HybrisManager::handleForType(int sensorType) const
{
    int index = indexForType(sensorType);
    return (index < 0) ? -1 : m_sensorArray[index].handle;
}

sensors_event_t *HybrisManager::eventForHandle(int handle) const
{
    sensors_event_t *event = 0;
    int index = indexForHandle(handle);
    if (index != -1) {
        event = &m_sensorState[index].m_fallbackEvent;
    }
    return event;
}

int HybrisManager::indexForHandle(int handle) const
{
    int index = m_indexOfHandle.value(handle, -1);
    if (index == -1)
        sensordLogW("HYBRIS CTL invalid sensor handle: %d", handle);
    return index;
}

int HybrisManager::indexForType(int sensorType) const
{
    int index = m_indexOfType.value(sensorType, -1);
    if (index == -1)
        sensordLogW("HYBRIS CTL invalid sensor type: %d", sensorType);
    return index;
}

void HybrisManager::startReader(HybrisAdaptor *adaptor)
{
    if (m_registeredAdaptors.values().contains(adaptor)) {
        sensordLogD() << "activating " << adaptor->name() << adaptor->m_sensorHandle;
        if (!setActive(adaptor->m_sensorHandle, true)) {
            sensordLogW() <<Q_FUNC_INFO<< "failed";
            adaptor->setValid(false);
        }
    }
}

void HybrisManager::stopReader(HybrisAdaptor *adaptor)
{
    if (m_registeredAdaptors.values().contains(adaptor)) {
            sensordLogD() << "deactivating " << adaptor->name();
            if (!setActive(adaptor->m_sensorHandle, false)) {
                sensordLogW() <<Q_FUNC_INFO<< "failed";
            }
    }
}

void HybrisManager::processSample(const sensors_event_t& data)
{
    foreach (HybrisAdaptor *adaptor, m_registeredAdaptors.values(data.type)) {
        if (adaptor->isRunning()) {
            adaptor->processSample(data);
        }
    }
}

void HybrisManager::registerAdaptor(HybrisAdaptor *adaptor)
{
    if (!m_registeredAdaptors.values().contains(adaptor) && adaptor->isValid()) {
        m_registeredAdaptors.insertMulti(adaptor->m_sensorType, adaptor);
    }
}

float HybrisManager::getMaxRange(int handle) const
{
    float range = 0;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];

        range = sensor->maxRange;
        sensordLogT("HYBRIS CTL getMaxRange(%d=%s) -> %g",
                    sensor->handle, sensorTypeName(sensor->type), range);
    }

    return range;
}

float HybrisManager::getResolution(int handle) const
{
    float resolution = 0;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];

        resolution = sensor->resolution;
        sensordLogT("HYBRIS CTL getResolution(%d=%s) -> %g",
                    sensor->handle, sensorTypeName(sensor->type), resolution);
    }

    return resolution;
}

int HybrisManager::getMinDelay(int handle) const
{
    int delay = 0;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        delay = state->m_minDelay;
        sensordLogT("HYBRIS CTL getMinDelay(%d=%s) -> %d",
                    sensor->handle, sensorTypeName(sensor->type), delay);
    }

    return delay;
}

int HybrisManager::getMaxDelay(int handle) const
{
    int delay = 0;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        delay = state->m_maxDelay;
        sensordLogT("HYBRIS CTL getMaxDelay(%d=%s) -> %d",
                    sensor->handle, sensorTypeName(sensor->type), delay);
    }

    return delay;
}

int HybrisManager::getDelay(int handle) const
{
    int delay = 0;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        delay = state->m_delay;
        sensordLogT("HYBRIS CTL getDelay(%d=%s) -> %d",
                    sensor->handle, sensorTypeName(sensor->type), delay);
    }

    return delay;
}

bool HybrisManager::setDelay(int handle, int delay_ms, bool force)
{
    bool success = false;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        if (!force && state->m_delay == delay_ms) {
            sensordLogT("HYBRIS CTL setDelay(%d=%s, %d) -> no-change",
                        sensor->handle, sensorTypeName(sensor->type), delay_ms);
        } else {
            int64_t delay_ns = delay_ms * 1000LL * 1000LL;
#ifdef USE_BINDER
            int error = 0;
            if (m_sensors) {
                auto ret = m_sensors->batch(sensor->handle, delay_ns, 0);
                if (!ret.isOk()) {
                    sensordLogW() << "Set delay failed status " << ret.description().c_str();
                    return false;
                }
            } else {
                GBinderLocalRequest *req = gbinder_client_new_request(m_client);
                GBinderRemoteReply *reply;
                GBinderReader reader;
                GBinderWriter writer;
                int32_t status;

                gbinder_local_request_init_writer(req, &writer);

                gbinder_writer_append_int32(&writer, sensor->handle);
                gbinder_writer_append_int64(&writer, delay_ns);
                gbinder_writer_append_int64(&writer, 0);

                reply = gbinder_client_transact_sync_reply(m_client, BATCH, req, &status);
                gbinder_local_request_unref(req);

                if (status != GBINDER_STATUS_OK) {
                    sensordLogW() << "Set delay failed status " << status;
                    return false;
                }
                gbinder_remote_reply_init_reader(reply, &reader);
                gbinder_reader_read_int32(&reader, &status);
                gbinder_reader_read_int32(&reader, &error);

                gbinder_remote_reply_unref(reply);
            }
#else
            int error = m_halDevice->setDelay(m_halDevice, sensor->handle, delay_ns);
#endif
            if (error) {
                sensordLogW("HYBRIS CTL setDelay(%d=%s, %d) -> %d=%s",
                            sensor->handle, sensorTypeName(sensor->type), delay_ms,
                            error, strerror(error));
            } else {
                sensordLogD("HYBRIS CTL setDelay(%d=%s, %d) -> success",
                            sensor->handle, sensorTypeName(sensor->type), delay_ms);
                state->m_delay = delay_ms;
                success = true;
            }
        }
    }

    return success;
}

bool HybrisManager::getActive(int handle) const
{
    bool active = false;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        active = (state->m_active > 0);
        sensordLogT("HYBRIS CTL getActive(%d=%s) -> %s",
                    sensor->handle, sensorTypeName(sensor->type),
                    active ? "true" : "false");
    }
    return active;
}

bool HybrisManager::setActive(int handle, bool active)
{
    bool success = false;
    int index = indexForHandle(handle);

    if (index != -1) {
        const struct sensor_t *sensor = &m_sensorArray[index];
        HybrisSensorState     *state  = &m_sensorState[index];

        if (state->m_active == active) {
            sensordLogT("HYBRIS CTL setActive%d=%s, %s) -> no-change",
                        sensor->handle, sensorTypeName(sensor->type), active ? "true" : "false");
            success = true;
        } else {
#ifdef USE_BINDER
            if (active && state->m_delay != -1) {
                sensordLogD("HYBRIS CTL FORCE PRE UPDATE %i, %s", sensor->handle, sensorTypeName(sensor->type));
                int delay_ms = state->m_delay;
                state->m_delay = -1;
                setDelay(handle, delay_ms, true);
            }
            int error = 0;
            if (m_sensors != nullptr) {
                auto ret = m_sensors->activate(sensor->handle, active);
                if (!ret.isOk()) {
                    sensordLogW() << "Activate failed status " << ret.description().c_str();
                    return false;
                }
            } else {
                GBinderLocalRequest *req = gbinder_client_new_request(m_client);
                GBinderRemoteReply *reply;
                GBinderReader reader;
                GBinderWriter writer;
                int32_t status;

                gbinder_local_request_init_writer(req, &writer);

                gbinder_writer_append_int32(&writer, sensor->handle);
                gbinder_writer_append_int32(&writer, active);

                reply = gbinder_client_transact_sync_reply(m_client, ACTIVATE, req, &status);
                gbinder_local_request_unref(req);

                if (status != GBINDER_STATUS_OK) {
                    sensordLogW() << "Activate failed status " << status;
                    return false;
                }
                gbinder_remote_reply_init_reader(reply, &reader);
                gbinder_reader_read_int32(&reader, &status);
                gbinder_reader_read_int32(&reader, &error);

                gbinder_remote_reply_unref(reply);
            }
#else
            int error = m_halDevice->activate(m_halDevice, sensor->handle, active);
#endif
            if (error) {
                sensordLogW("HYBRIS CTL setActive%d=%s, %s) -> %d=%s",
                            sensor->handle, sensorTypeName(sensor->type), active ? "true" : "false",
                            error, strerror(error));
            } else {
                sensordLogD("HYBRIS CTL setActive%d=%s, %s) -> success",
                            sensor->handle, sensorTypeName(sensor->type), active ? "true" : "false");
                state->m_active = active;
                success = true;
            }
#ifndef USE_BINDER
            if (state->m_active == true && state->m_delay != -1) {
                sensordLogD("HYBRIS CTL FORCE DELAY UPDATE");
                int delay_ms = state->m_delay;
                state->m_delay = -1;
                setDelay(handle, delay_ms, false);
            }
#endif
        }
    }
    return success;
}

#ifdef USE_BINDER
/**
 * pollEvents is only called during initialization and after that from pollEventsCallback
 * triggered by binder reply so there is only maximum of one active poll at all times
 */
void HybrisManager::pollEvents()
{
    if (m_client) {
        GBinderLocalRequest *req = gbinder_client_new_request(m_client);

        req = gbinder_local_request_append_int32(req, 16); // Same number as for HAL

        m_pollTransactId = gbinder_client_transact(m_client, POLL, 0, req, pollEventsCallback, 0, this);
        gbinder_local_request_unref(req);
    }
}

void HybrisManager::pollEventsCallback(
    GBinderClient* /*client*/,
    GBinderRemoteReply* reply,
    int status,
    void* userData)
{
    HybrisManager *manager = static_cast<HybrisManager *>(userData);
    bool blockSuspend = false;
    bool errorInInput = false;
    GBinderReader reader;
    int32_t readerStatus;
    int32_t result;
    sensors_event_t *buffer;

    manager->m_pollTransactId = 0;

    if (status != GBINDER_STATUS_OK) {
        sensordLogW() << "Poll failed status " << status;
        // In case of binder failure sleep a little before attempting a new poll
        struct timespec ts = { 0, 50 * 1000 * 1000 }; // 50 ms
        do { } while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
    } else {
        // Read sensor events from reply
        gbinder_remote_reply_init_reader(reply, &reader);
        gbinder_reader_read_int32(&reader, &readerStatus);
        gbinder_reader_read_int32(&reader, &result);
        gsize structSize = 0;
        gsize eventCount = 0;

        buffer = (sensors_event_t *)gbinder_reader_read_hidl_vec(&reader, &eventCount , &structSize);
        manager->processEvents(buffer, eventCount, blockSuspend, errorInInput);

        if (blockSuspend) {
            ObtainTemporaryWakeLock();
        }
    }
    // Initiate new poll
    manager->pollEvents();
}

template<typename EnumType>
constexpr typename std::underlying_type<EnumType>::type asBaseType(EnumType value) {
    return static_cast<typename std::underlying_type<EnumType>::type>(value);
}

// Used internally by the framework to wake the Event FMQ. These values must start after
// the last value of EventQueueFlagBits (from sensorservice/SensorDevice.cpp)
enum EventQueueFlagBitsInternal : uint32_t {
    INTERNAL_WAKE =  1 << 16,
};

void *HybrisManager::fmqEventReaderThread(void *aptr)
{
    using android::hardware::sensors::V1_0::Event;
    using android::hardware::sensors::V2_0::EventQueueFlagBits;
    //using android::hardware::sensors::V1_0::implementation::convertToSensorEvent;

    HybrisManager *manager = static_cast<HybrisManager *>(aptr);
    static const size_t numEvents = 16;
    std::array<Event, numEvents> eventBuffer;

    /* Async cancellation, but disabled */
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);
    /* Leave INT/TERM signal processing up to the main thread */
    sigset_t ss;
    sigemptyset(&ss);
    sigaddset(&ss, SIGINT);
    sigaddset(&ss, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &ss, 0);

    /* Loop until explicitly canceled */
    for (;;) {
        int numberOfEvents = 0;
        size_t availableEvents = manager->m_eventQueue->availableToRead();

        if (availableEvents == 0) {
            uint32_t eventFlagState = 0;

            /* Async cancellation point at android hal poll() */
            pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);

            /* Wait for events to become available so read() can
             * be called with correct amount of event */
            manager->m_eventQueueFlag->wait(asBaseType(EventQueueFlagBits::READ_AND_PROCESS) |
                                asBaseType(INTERNAL_WAKE), &eventFlagState);

            pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);

            availableEvents = manager->m_eventQueue->availableToRead();
        }

        size_t eventsToRead = std::min({availableEvents, eventBuffer.size()});
        if (eventsToRead > 0) {
            if (manager->m_eventQueue->read(eventBuffer.data(), eventsToRead)) {
                /* Notify the Sensors HAL that sensor events have been read. This is required to support
                 * the use of writeBlocking by the Sensors HAL. */
                manager->m_eventQueueFlag->wake(asBaseType(EventQueueFlagBits::EVENTS_READ));
                numberOfEvents = eventsToRead;
                /* sensordLogW() << "Read " << eventsToRead << "sensor events"; */
            } else {
                sensordLogW() << "Failed to read " << eventsToRead << " events, currently "
                              << availableEvents << " events available";

                /* Rate limit in read() error situations */
                struct timespec ts = { 1, 0 }; // 1000 ms
                do { } while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
                continue;
            }
        } else {
            continue;
        }

        /* Process received events */
        bool blockSuspend = false;
        bool errorInInput = false;

        static_assert(sizeof(Event) == sizeof(sensors_event_t), "wrong size");
        manager->processEvents(reinterpret_cast<sensors_event_t*>(eventBuffer.data()), numberOfEvents, blockSuspend, errorInInput);

        /* Suspend proof sensor reporting that could occur in display off */
        if (blockSuspend) {
            ObtainTemporaryWakeLock();
        }
        /* Rate limit after receiving erraneous events */
        if (errorInInput) {
            struct timespec ts = { 0, 50 * 1000 * 1000 }; // 50 ms
            do { } while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
        }
    }
    return 0;
}

#else

void *HybrisManager::halEventReaderThread(void *aptr)
{
    HybrisManager *manager = static_cast<HybrisManager *>(aptr);
    static const size_t numEvents = 16;
    sensors_event_t buffer[numEvents];

    /* Async cancellation, but disabled */
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, 0);
    /* Leave INT/TERM signal processing up to the main thread */
    sigset_t ss;
    sigemptyset(&ss);
    sigaddset(&ss, SIGINT);
    sigaddset(&ss, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &ss, 0);
    /* Loop until explicitly canceled */
    for (;;) {
        /* Async cancellation point at android hal poll() */
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, 0);
        int numberOfEvents = manager->m_halDevice->poll(manager->m_halDevice, buffer, numEvents);
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, 0);
        /* Rate limit in poll() error situations */
        if (numberOfEvents < 0) {
            sensordLogW() << "android device->poll() failed" << strerror(-numberOfEvents);
            struct timespec ts = { 1, 0 }; // 1000 ms
            do { } while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
            continue;
        }
        /* Process received events */
        bool blockSuspend = false;
        bool errorInInput = false;
        manager->processEvents(buffer, numberOfEvents, blockSuspend, errorInInput);

        /* Suspend proof sensor reporting that could occur in display off */
        if (blockSuspend) {
            ObtainTemporaryWakeLock();
        }
        /* Rate limit after receiving erraneous events */
        if (errorInInput) {
            struct timespec ts = { 0, 50 * 1000 * 1000 }; // 50 ms
            do { } while (nanosleep(&ts, &ts) == -1 && errno == EINTR);
        }
    }
    return 0;
}
#endif

void HybrisManager::processEvents(const sensors_event_t *buffer, int numberOfEvents, bool &blockSuspend, bool &errorInInput)
{
    for (int i = 0; i < numberOfEvents; i++) {
        const sensors_event_t& data = buffer[i];

        sensordLogT("HYBRIS EVE %s", sensorTypeName(data.type));

        /* Got data -> Clear the no longer needed fallback event */
        sensors_event_t *fallback = eventForHandle(data.sensor);
        if (fallback && fallback->type == data.type && fallback->sensor == data.sensor) {
            fallback->type = fallback->sensor = 0;
        }

#ifdef USE_BINDER
        Q_UNUSED(errorInInput);
#else
        if (data.version != sizeof(sensors_event_t)) {
            sensordLogW()<< QString("incorrect event version (version=%1, expected=%2").arg(data.version).arg(sizeof(sensors_event_t));
            errorInInput = true;
        }
#endif

        if (data.type == SENSOR_TYPE_PROXIMITY) {
            blockSuspend = true;
        }
        // FIXME: is this thread safe?
        processSample(data);
    }
}

/* ========================================================================= *
 * HybrisAdaptor
 * ========================================================================= */

HybrisAdaptor::HybrisAdaptor(const QString& id, int type)
    : DeviceAdaptor(id)
    , m_inStandbyMode(false)
    , m_isRunning(false)
    , m_shouldBeRunning(false)
    , m_sensorHandle(-1)
    , m_sensorType(type)
{
    m_sensorHandle = hybrisManager()->handleForType(m_sensorType);
    if (m_sensorHandle == -1) {
        sensordLogW() << Q_FUNC_INFO <<"no such sensor" << id;
        setValid(false);
        return;
    }

    hybrisManager()->registerAdaptor(this);
}

HybrisAdaptor::~HybrisAdaptor()
{
}

void HybrisAdaptor::init()
{
    introduceAvailableDataRange(DataRange(minRange(), maxRange(), resolution()));
    introduceAvailableInterval(DataRange(minInterval(), maxInterval(), 0));
}

void HybrisAdaptor::sendInitialData()
{
    // virtual dummy
    // used for ps/als initial value hacks
}

bool HybrisAdaptor::writeToFile(const QByteArray& path, const QByteArray& content)
{
    sensordLogT() << "Writing to '" << path << ": " << content;
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly))
    {
        sensordLogW() << "Failed to open '" << path << "': " << file.errorString();
        return false;
    }
    if (file.write(content.constData(), content.size()) == -1)
    {
        sensordLogW() << "Failed to write to '" << path << "': " << file.errorString();
        file.close();
        return false;
    }

    file.close();
    return true;
}

/* ------------------------------------------------------------------------- *
 * range
 * ------------------------------------------------------------------------- */

qreal HybrisAdaptor::minRange() const
{
    return 0;
}

qreal HybrisAdaptor::maxRange() const
{
    return hybrisManager()->getMaxRange(m_sensorHandle);
}

qreal HybrisAdaptor::resolution() const
{
    return hybrisManager()->getResolution(m_sensorHandle);
}

/* ------------------------------------------------------------------------- *
 * interval
 * ------------------------------------------------------------------------- */

unsigned int HybrisAdaptor::minInterval() const
{
    return hybrisManager()->getMinDelay(m_sensorHandle);
}

unsigned int HybrisAdaptor::maxInterval() const
{
    return hybrisManager()->getMaxDelay(m_sensorHandle);
}

unsigned int HybrisAdaptor::interval() const
{
    return hybrisManager()->getDelay(m_sensorHandle);
}

bool HybrisAdaptor::setInterval(const unsigned int value, const int sessionId)
{
    Q_UNUSED(sessionId);

    bool ok = hybrisManager()->setDelay(m_sensorHandle, value, false);

    if (!ok) {
        sensordLogW() << Q_FUNC_INFO << "setInterval not ok";
    } else {
        /* If we have not yet received sensor data, apply fallback value */
        sensors_event_t *fallback = hybrisManager()->eventForHandle(m_sensorHandle);
        if (fallback && fallback->sensor == m_sensorHandle && fallback->type == m_sensorType) {
            sensordLogT("HYBRIS FALLBACK type:%s sensor:%d",
                        sensorTypeName(fallback->type),
                        fallback->sensor);
            processSample(*fallback);
            fallback->sensor = fallback->type = 0;
        }

        sendInitialData();
    }

    return ok;
}

unsigned int HybrisAdaptor::evaluateIntervalRequests(int& sessionId) const
{
    if (m_intervalMap.size() == 0)
    {
        sessionId = -1;
        return defaultInterval();
    }

    // Get the smallest positive request, 0 is reserved for HW wakeup
    QMap<int, unsigned int>::const_iterator it = m_intervalMap.constBegin();
    unsigned int highestValue = it.value();
    int winningSessionId = it.key();

    for (++it; it != m_intervalMap.constEnd(); ++it)
    {
        if (((it.value() < highestValue) && (it.value() > 0)) || highestValue == 0) {
            highestValue = it.value();
            winningSessionId = it.key();
        }
    }

    sessionId = winningSessionId;
    return highestValue > 0 ? highestValue : defaultInterval();
}

/* ------------------------------------------------------------------------- *
 * start/stop adaptor
 * ------------------------------------------------------------------------- */

bool HybrisAdaptor::startAdaptor()
{
    return isValid();
}

void HybrisAdaptor::stopAdaptor()
{
    if (getAdaptedSensor()->isRunning())
        stopSensor();
}

/* ------------------------------------------------------------------------- *
 * start/stop sensor
 * ------------------------------------------------------------------------- */

bool HybrisAdaptor::isRunning() const
{
    return m_isRunning;
}

void HybrisAdaptor::evaluateSensor()
{
    // Get listener object
    AdaptedSensorEntry *entry = getAdaptedSensor();
    if (entry == NULL) {
        sensordLogW() << Q_FUNC_INFO << "Sensor not found: " << name();
        return;
    }

    // Check policy
    bool runningAllowed = (deviceStandbyOverride() || !m_inStandbyMode);

    // Target state
    bool startRunning = m_shouldBeRunning && runningAllowed;

    if (m_isRunning != startRunning) {
        if ((m_isRunning = startRunning)) {
            hybrisManager()->startReader(this);
            if (entry->addReference() == 1) {
                entry->setIsRunning(true);
            }

            /* If we have not yet received sensor data, apply fallback value */
            sensors_event_t *fallback = hybrisManager()->eventForHandle(m_sensorHandle);
            if (fallback && fallback->sensor == m_sensorHandle && fallback->type == m_sensorType) {
                sensordLogT("HYBRIS FALLBACK type:%s sensor:%d",
                            sensorTypeName(fallback->type),
                            fallback->sensor);
                processSample(*fallback);
                fallback->sensor = fallback->type = 0;
            }
        } else {
            if (entry->removeReference() == 0) {
                entry->setIsRunning(false);
            }
            hybrisManager()->stopReader(this);
        }
        sensordLogT() << Q_FUNC_INFO << "entry" << entry->name()
                      << "refs:" << entry->referenceCount() << "running:" << entry->isRunning();
    }
}

bool HybrisAdaptor::startSensor()
{
    // Note: This is overloaded and called by each HybrisXxxAdaptor::startSensor()
    if (!m_shouldBeRunning) {
        m_shouldBeRunning = true;
        sensordLogT("%s m_shouldBeRunning = %d", sensorTypeName(m_sensorType), m_shouldBeRunning);
        evaluateSensor();
    }
    return true;
}

void HybrisAdaptor::stopSensor()
{
    // Note: This is overloaded and called by each HybrisXxxAdaptor::stopSensor()
    if (m_shouldBeRunning) {
        m_shouldBeRunning = false;
        sensordLogT("%s m_shouldBeRunning = %d", sensorTypeName(m_sensorType), m_shouldBeRunning);
        evaluateSensor();
    }
}

bool HybrisAdaptor::standby()
{
    if (!m_inStandbyMode) {
        m_inStandbyMode = true;
        sensordLogT("%s m_inStandbyMode = %d", sensorTypeName(m_sensorType), m_inStandbyMode);
        evaluateSensor();
    }
    return true;
}

bool HybrisAdaptor::resume()
{
    if (m_inStandbyMode) {
        m_inStandbyMode = false;
        sensordLogT("%s m_inStandbyMode = %d", sensorTypeName(m_sensorType), m_inStandbyMode);
        evaluateSensor();
    }
    return true;
}
