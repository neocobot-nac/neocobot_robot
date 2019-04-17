/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     safemap.h
*  @brief    线程安全map容器
*
*  @detail   定义了线程安全map容器模块
*			 1.提供线程安全容器,存放接收到的任务返回值
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
#ifndef SAFEMAP_H
#define SAFEMAP_H

#include <map>
#include <mutex>
#include <condition_variable>

template<class Key, class T>
class threadsafe_map
{
public:
	void insert(const Key &inputKey, const T &inputValue) {
		std::unique_lock<std::mutex> ulk(mut);
		the_map.insert(std::pair<Key, T>(inputKey, inputValue));
		ulk.unlock();
		cond.notify_one();
	}

	bool empty() const {
		std::unique_lock<std::mutex> ulk(mut);
		return the_map.empty();
	}

	bool try_get(const Key &inputKey, T &outputValue) {
		std::unique_lock<std::mutex> ulk(mut);
		typename std::map<Key, T>::iterator it;
		it = the_map.find(inputKey);
		if (the_map.end() == it) {
			return false;
		}
		outputValue = it->second;
		return true;
	}

	void wait_and_get(const Key &inputKey, T &outputValue) {
		std::unique_lock<std::mutex> ulk(mut);
		typename std::map<Key, T>::iterator it;
		while (the_map.end() == (it = the_map.find(inputKey))) {
			cond.wait(ulk);
		}
		outputValue = it->second;
	}

	void wait_next_insert() {
		std::unique_lock<std::mutex> ulk(mut);
		cond.wait(ulk);
	}

	void erase(const Key &inputKey) {
		std::unique_lock<std::mutex> ulk(mut);
		the_map.erase(inputKey);
	}

	size_t size(){
		std::unique_lock<std::mutex> ulk(mut);
		return the_map.size();
	}

private:
	std::map<Key, T> the_map;
	std::mutex mut;
	std::condition_variable cond;

};

#endif