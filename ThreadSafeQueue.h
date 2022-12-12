#ifndef SAFE_QUEUE
#define SAFE_QUEUE

#include <queue>


#if defined(_STM32_DEF_)

// A simple, non-threadsafe queue for non-multithreading CPUs.
template <class T>
class SafeQueue
{
    public:
        SafeQueue(int Size) : q(),NumElements(0)
        {}

        // Add an element to the queue.
        void Enqueue(T t)
        {
            q.push(t);
            NumElements++;
        }

        // Get the "front"-element.
        // If the queue is empty, wait till a element is avaiable.
        bool Dequeue(T* pVal)
        {
            if(!q.empty())
            {
                *pVal = q.front();
                q.pop();
                NumElements--;
                return true;
            }
            return false;
        }

        int NumElements;

private:
  std::queue<T> q;
};
#endif


#if defined(xxx)
// A threadsafe-queue for CPUs supporting std::mutex
// Inspired by: https://stackoverflow.com/questions/15278343/c11-thread-safe-queue
#include <mutex>

template <class T>
class SafeQueue
{
    public:
        SafeQueue(int Size) : q(), m(), NumElements(0)
        {}

        // Add an element to the queue.
        void Enqueue(T t)
        {
            std::lock_guard<std::mutex> lock(m);
            q.push(t);
            NumElements++;
        }

        // Get the "front"-element.
        // If the queue is empty, wait till a element is avaiable.
        bool Dequeue(T* pVal)
        {
            std::unique_lock<std::mutex> lock(m);
            if(!q.empty())
            {
                *pVal = q.front();
                q.pop();
                NumElements--;
                return true;
            }
            return false;
        }

        int NumElements;

private:
  std::queue<T> q;
  mutable std::mutex m;
};

#endif


#if defined(_ESP32_)

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

template <class T>
class SafeQueue
{
    public:
        SafeQueue(int Size=16) : NumElements(0)
        {
            Queue = xQueueCreate( Size, sizeof( T ) );
        }

        // Add an element to the queue.
        // ISR, absolutely no printing to serial!!
        void Enqueue(T t)
        {
            xQueueSendFromISR(Queue, &t, 0);
            NumElements++;
        }

        // Get the "front"-element.
        // If the queue is empty, wait till a element is avaiable.
        // ISR, absolutely no printing to serial!!
        bool Dequeue(T* pVal)
        {
            if(xQueueReceive(Queue, pVal, 0))
            {
                NumElements--;
                return true;
            }
            return false;
        }

        int NumElements;

private:
  QueueHandle_t Queue;
};

#endif
#endif
