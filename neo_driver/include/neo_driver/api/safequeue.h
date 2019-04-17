/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     safequeue.h
*  @brief    线程安全queue
*
*  @detail   定义了线程安全queue
*			 1.提供线程安全队列,存放即将发送的任务
*
*  @author   Huang Shaojie
*  @email    shaojie@neocobot.com
*  @version  0.2.0
*  @date     2019/01/17
*
*----------------------------------------------------------------------------
*  Change History :
*  <Date>     | <Version> | <Author>       | <Description>
*----------------------------------------------------------------------------
*  2019/01/17 | 0.2.0     | Huang Shaojie  | 重组并定义功能
*----------------------------------------------------------------------------
*****************************************************************************/
#ifndef SAFEQUEUE_H
#define SAFEQUEUE_H

#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>

template<typename T>
class threadsafe_queue
{
public:
	threadsafe_queue() {}
	~threadsafe_queue() {}

	void push(T new_data)
	{
		std::lock_guard<std::mutex> lk(mut);            // 1.全局加锁
		data_queue.push(std::move(new_data));           // 2.push时独占锁
		cond.notify_one();
	}

	void wait_and_pop(T& val)
	{
		std::unique_lock<std::mutex> ulk(mut);                    // 3.全局加锁
		cond.wait(ulk, [this]() { return !data_queue.empty(); });  // 4.front 和 pop_front时独占锁
		val = std::move(data_queue.front());
		data_queue.pop();
	}

	std::shared_ptr<T> wait_and_pop()
	{
		std::unique_lock<std::mutex> ulk(mut);
		cond.wait(ulk, [this]() { return !data_queue.empty(); });
		std::shared_ptr<T> val(std::make_shared<T>(std::move(data_queue.front())));
		data_queue.pop();
		return val;
	}

	bool try_pop(T& val)
	{
		std::lock_guard<std::mutex> lk(mut);
		if (data_queue.empty())
			return false;
		val = std::move(data_queue.front());
		data_queue.pop();
		return true;
	}

	std::shared_ptr<T> try_pop()
	{
		std::shared_ptr<T> val;
		std::lock_guard<std::mutex> lk(mut);
		if (data_queue.empty())
			return val;
		val = std::make_shared<T>(std::move(data_queue.front()));
		data_queue.pop();
		return val;
	}

	bool empty()
	{
		std::lock_guard<std::mutex> lk(mut);
		return data_queue.empty();
	}

private:
	std::queue<T> data_queue;
	std::mutex mut;
	std::condition_variable cond;
};

#endif