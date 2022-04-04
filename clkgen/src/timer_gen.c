#include <stdint.h>
#include <unistd.h>
#include <string.h>
#ifndef __APPLE__
#include <sys/timerfd.h>
#else
#include <dispatch/dispatch.h>
#include <stdlib.h>
#endif
#include <pthread.h>
#include <poll.h>
#include <stdio.h>

#include "timer_gen.h"

#define MAX_TIMER_COUNT 1000


struct timer_node
{
#ifndef __APPLE__
    int fd;
    time_handler callback;
    void *user_data;
    unsigned long long int interval;
    t_timer type;
#else
    bool active;
    dispatch_source_t timer;
#endif
    struct timer_node *next;
};

#ifndef __APPLE__
static void *_timer_thread(void *data);
static pthread_t g_thread_id;
#else
static void sigtrap(int sig)
{
    finalize();
}

static bool queue_initd = false;
static dispatch_queue_t queue;
#endif
static struct timer_node *g_head = NULL;

int initialize()
{
#ifndef __APPLE__
    if (pthread_create(&g_thread_id, NULL, _timer_thread, NULL))
    {
        /*Thread creation failed*/
        return -1;
    }

    return 1;
#else
    if (!queue_initd)
    {
        queue_initd = true;
        signal(SIGINT, sigtrap);
        queue = dispatch_queue_create("timer_gen_timer_queue", 0);
    }
    return 1;
#endif
}

size_t start_timer(unsigned long long int interval, time_handler handler, t_timer type, void *user_data)
{
    struct timer_node *new_node = NULL;

    new_node = (struct timer_node *)malloc(sizeof(struct timer_node));

    if (new_node == NULL)
        return 0;

#ifndef __APPLE__
    struct itimerspec new_value;

    new_node->callback = handler;
    new_node->user_data = user_data;
    new_node->interval = interval;
    new_node->type = type;

    new_node->fd = timerfd_create(CLOCK_REALTIME, 0);

    if (new_node->fd == -1)
    {
        free(new_node);
        return 0;
    }

    new_value.it_value.tv_sec = interval / 1000000000;
    new_value.it_value.tv_nsec = (interval % 1000000000);

    if (type == TIMER_PERIODIC)
    {
        new_value.it_interval.tv_sec = interval / 1000000000;
        new_value.it_interval.tv_nsec = (interval % 1000000000);
    }
    else
    {
        new_value.it_interval.tv_sec = 0;
        new_value.it_interval.tv_nsec = 0;
    }

    timerfd_settime(new_node->fd, 0, &new_value, NULL);
#else
    new_node->timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, queue);
    dispatch_source_set_event_handler(new_node->timer, ^{
      handler((size_t)new_node, user_data);
    });
    dispatch_source_set_cancel_handler(new_node->timer, ^{
      dispatch_release(new_node->timer);
      dispatch_release(queue);
    });
    dispatch_time_t start = dispatch_time(DISPATCH_TIME_NOW, 0);
    dispatch_source_set_timer(new_node->timer, start, interval, 0);
    dispatch_resume(new_node->timer);
    new_node->active = true;
#endif
    /*Inserting the timer node into the list*/
    new_node->next = g_head;
    g_head = new_node;

    return (size_t)new_node;
}

size_t update_timer(size_t timer_id, unsigned long long interval, t_timer type)
{
    struct timer_node *node = (struct timer_node *)timer_id;
    if (node == NULL) // on error, invalid timer ID
        return (size_t)NULL;
#ifndef __APPLE__
    struct itimerspec new_value;

    new_value.it_value.tv_sec = interval / 1000000000;
    new_value.it_value.tv_nsec = (interval % 1000000000);

    if (type == TIMER_PERIODIC)
    {
        new_value.it_interval.tv_sec = interval / 1000000000;
        new_value.it_interval.tv_nsec = (interval % 1000000000);
    }
    else
    {
        new_value.it_interval.tv_sec = 0;
        new_value.it_interval.tv_nsec = 0;
    }

    timerfd_settime(node->fd, 0, &new_value, NULL);
#else
    dispatch_time_t start = dispatch_time(DISPATCH_TIME_NOW, 0);
    dispatch_suspend(node->timer);
    node->active = false;
    dispatch_source_set_timer(node->timer, start, interval, 0);
    dispatch_resume(node->timer);
    node->active = true;
#endif // __APPLE__
    return (size_t) node;
}

void stop_timer(size_t timer_id)
{
    struct timer_node *tmp = NULL;
    struct timer_node *node = (struct timer_node *)timer_id;

    if (node == NULL)
        return;
#ifndef __APPLE__
    close(node->fd);
#else
    if (node->active == false)
        dispatch_source_cancel(node->timer);
    node->active = false;
#endif

    if (node == g_head)
    {
        g_head = g_head->next;
    }
    else
    {

        tmp = g_head;

        while (tmp && tmp->next != node)
            tmp = tmp->next;

        if (tmp)
        {
            /*tmp->next can not be NULL here.*/
            tmp->next = tmp->next->next;
        }
    }
    if (node)
        free(node);
}

void finalize()
{

    while (g_head)
        stop_timer((size_t)g_head);
#ifndef __APPLE__
    pthread_cancel(g_thread_id);
    pthread_join(g_thread_id, NULL);
#endif
}
#ifndef __APPLE__
struct timer_node *_get_timer_from_fd(int fd)
{
    struct timer_node *tmp = g_head;

    while (tmp)
    {
        if (tmp->fd == fd)
            return tmp;

        tmp = tmp->next;
    }
    return NULL;
}

void *_timer_thread(void *data)
{
    struct pollfd ufds[MAX_TIMER_COUNT] = {{0}};
    int iMaxCount = 0;
    struct timer_node *tmp = NULL;
    int read_fds = 0, i, s;
    uint64_t exp;

    while (1)
    {
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_testcancel();
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

        iMaxCount = 0;
        tmp = g_head;

        memset(ufds, 0, sizeof(struct pollfd) * MAX_TIMER_COUNT);
        while (tmp)
        {
            ufds[iMaxCount].fd = tmp->fd;
            ufds[iMaxCount].events = POLLIN;
            iMaxCount++;

            tmp = tmp->next;
        }
        read_fds = poll(ufds, iMaxCount, 100);

        if (read_fds <= 0)
            continue;

        for (i = 0; i < iMaxCount; i++)
        {
            if (ufds[i].revents & POLLIN)
            {
                s = read(ufds[i].fd, &exp, sizeof(uint64_t));

                if (s != sizeof(uint64_t))
                    continue;

                tmp = _get_timer_from_fd(ufds[i].fd);

                if (tmp && tmp->callback)
                    tmp->callback((size_t)tmp, tmp->user_data);
            }
        }
    }

    return NULL;
}
#endif