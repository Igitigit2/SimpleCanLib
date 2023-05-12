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
#include "freertos/semphr.h"

template <class T>
class SafeQueue
{
    public:
        SafeQueue(int Size=16) 
        {
            Queue = xQueueCreate( Size, sizeof( T ) );
            NumElements = 0;
            QueueSema = xSemaphoreCreateCounting( 1, 0 );        
        }

        // Add an element to the queue.
        // ISR, absolutely no printing to serial!!
        void Enqueue(T t)
        {
            xSemaphoreTakeFromISR(QueueSema, 0);
            xQueueSendFromISR(Queue, &t, 0);
            NumElements++;
            xSemaphoreGiveFromISR(QueueSema, 0);
        }

        // Get the "front"-element.
        // If the queue is empty, wait till an element is avaiable.
        // ISR, absolutely no printing to serial!!
        bool Dequeue(T* pVal)
        {
            xSemaphoreTakeFromISR(QueueSema, 0);
            if(xQueueReceiveFromISR(Queue, pVal, 0))
            {
                NumElements--;
                xSemaphoreGiveFromISR(QueueSema, 0);
                return true;
            }
            xSemaphoreGiveFromISR(QueueSema, 0);
            return false;
        }

        int NumElements;

private:
  QueueHandle_t Queue;
  SemaphoreHandle_t QueueSema;
};

#endif

#if 0
template<class T>
T volatile_copy(T const volatile& source) {
    static_assert(std::is_trivially_copyable_v<T>, "Input must be trivially copyable");
    T dest;
    auto* dest_ptr = dynamic_cast<char*>(&dest);
    auto* source_ptr = dynamic_cast<char const volatile*>(&source);

    for(int i = 0; i < sizeof(T); i++) {
        dest_ptr[i] = source_ptr[i];
    }

    return dest;
}

template<class T>
void volatile_assign(T volatile& dest, T const& source) {
    static_assert(std::is_trivially_copyable_v<T>, "Input must be trivially copyable");
    auto* source_ptr = dynamic_cast<char*>(&source);
    auto* dest_ptr   = dynamic_cast<char volatile*>(&dest);

    for(int i = 0; i < sizeof(T); i++) {
        dest_ptr[i] = source_ptr[i];
    }
}
#endif

#endif
