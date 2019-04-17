/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     parser.h
*  @brief    xml消息解析和打包模块
*
*  @detail   目前版本的头文件定义了xml消息解析和打包模块
*			 1.提供了xml消息的打包和解包功能
*            2.提供了消息转化中的校验功能
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
#ifndef PARSER_H
#define PARSER_H

#define XML_LEFT_BRACKET "<"
#define XML_LEFT_SLASH_BRACKET "</"
#define XML_RIGHT_BRACKET ">"
#define XML_RIGHT_SLASH_BRACKET "/>"
#define XML_SLASH "/"
#define XML_SPACE " "

#include "argdefine.h"
#include <string.h>
#include <string>
#include <map>
#include <vector>

using std::string;
using std::map;
using std::to_string;


typedef enum XML_Event
{
	XMLEvent_Succeed	= 0,	// 无错误
	XMLEvent_Error,				// 错误

}XMLEvent;

/*
@brief xml标签
*/
class NeoXmlElement
{
private:
	string xmlElement;

	string xml_name;
	size_t xml_size_of_name;
	size_t xml_start_name_index;
	size_t xml_end_name_index;

	string xml_text;
	size_t xml_size_of_text;
	size_t xml_text_index;

	vector<NeoXmlElement> children;

private:
	/*
	@brief 更新标签索引
	*/
	void _updateXMLParam();

public:
	NeoXmlElement(string element);
	NeoXmlElement();
	~NeoXmlElement();

	/*
	@brief 设置标签名
	@param string name 标签名字符串
	*/
	void					setName(string name);

	/*
	@brief 设置标签内容
	@param string name 标签内容字符串
	*/
	void					setText(string text);

	/*
	@brief 链接标签内容
	@param string name 链接的标签内容字符串
	*/
	void					linkText(string text);

	/*
	@brief 获取标签字符串
	@return string 返回标签对应的字符串
	*/
	string					getElement();

	/*
	@brief 获取标签名
	@return string 返回标签名
	*/
	string					getName();

	/*
	@brief 获取标签内容
	@return string 返回标签内容
	*/
	string					getText();

	/*
	@brief 获取子标签
	@return vector<NeoXmlElement> 子标签容器
	*/
	vector<NeoXmlElement>	getChildren();

	/*
	@brief 添加子标签
	@param NeoXmlElement &element 子标签
	*/
	void					setChildren(NeoXmlElement &element);
};

/*
@brief xml文件
*/
class NeoXmlDocument
{
private:
	string			xmlDocument;
	NeoXmlElement	root;

private:
	/*
	@brief 从字符串中解析对应的标签
	@param string &s	需要解析的字符串
	@param string name	标签名
	@return NeoXmlElement 解析得到的标签类
	*/
	NeoXmlElement	_getElement(string &s, string name);

	/*
	@brief 从字符串中获取标签名容器
	@param string			&s		需要解析的字符串
	@param vector<string>	&name	标签名容器
	@return XMLEvent 解析事件
	*/
	XMLEvent		_getElementNameVector(string &s, vector<string> &name);

	/*
	@brief 递归获得所有标签
	@param NeoXmlElement &element 子标签
	@return XMLEvent 解析事件
	*/
	XMLEvent		_recursionParse(NeoXmlElement &element);

public:
	NeoXmlDocument();
	~NeoXmlDocument();

	/*
	@brief 通过字符串获取所有标签
	@param string &s 字符串
	@return XMLEvent 解析事件
	*/
	XMLEvent		parse(string &s);

	/*
	@brief 获取xml根标签
	@return NeoXmlElement 根标签
	*/
	NeoXmlElement	getRoot();

	/*
	@brief 通过标签根生成字符串
	@param NeoXmlElement &root 标签根
	@return XMLEvent 解析事件
	*/
	XMLEvent		generate(NeoXmlElement &root);

	/*
	@brief 获取根标签对应的字符串
	@return string 字符串
	*/
	string			getString();

};

/*
@brief 校验模块
*/
class CheckSum
{
private:
	int					sum;
	vector<string>		_element;

public:
	CheckSum();
	~CheckSum();

	/*
	@brief 添加校验元素
	@param string element 需要校验的元素
	*/
	void	AddElement(string element);

	/*
	@brief 根据已添加的元素计算校验值
	@return int 返回校验值
	*/
	int		CalcSum();

};


/*
@brief xml消息解析模块
*/
class Parser
{
public:
	/*
	@brief 打包成注册消息
	@param string				&msg	打包后的消息
	@param map<string, string>	&data	打包前参数
	@return RobotEvent 事件
	*/
	RobotEvent wrap_license(string &msg, map<string, string> &data);

	/*
	@brief 打包成客户端信息消息
	@param string				&msg	打包后的消息
	@param map<string, string>	&data	打包前参数
	@return RobotEvent 事件
	*/
	RobotEvent wrap_clientinfo(string &msg, map<string, string> &data);

	/*
	@brief 打包成心跳包消息
	@param string				&msg	打包后的消息
	@param map<string, string>	&data	打包前参数
	@return RobotEvent 事件
	*/
	RobotEvent wrap_heart(string &msg, map<string, string> &data);

	/*
	@brief 打包成一般功能消息
	@param string				&msg	打包后的消息
	@param map<string, string>	&data	打包前参数
	@return RobotEvent 事件
	*/
	RobotEvent wrap_msg(string &msg, map<string, string> &data);

	/*
	@brief 解包函数入口
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap(map<string, string> &data, string &msg);

	/*
	@brief 解包服务器信息消息
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap_ServerInfo(map<string, string> &data, string &msg);

	/*
	@brief 解包连接结果消息
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap_ConnResult(map<string, string> &data, string &msg);

	/*
	@brief 解包心跳包消息
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap_Heart(map<string, string> &data, string &msg);

	/*
	@brief 解包错误消息
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap_Error(map<string, string> &data, string &msg);

	/*
	@brief 解包一般消息
	@param map<string, string>	&data	解包后参数
	@param string				&msg	解包前消息
	@return RobotEvent 事件
	*/
	RobotEvent unwrap_Msg(map<string, string> &data, string &msg);

};

#endif
