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