#ifndef UTILS_NEW_H_
#define UTILS_NEW_H_

#include <ros/ros.h>

#include <cstring>
#include <cmath>
#include <assert.h>
#include <string>
#include <vector>
#include <cstdio>
#include <limits.h>
#include <exception>
#include <fstream>

#include "structs.h"

static double computeDistance(const double& x1,
                              const double& y1,
                              const double& x2,
                              const double& y2)
{
    double dx = x1 - x2;
    double dy = y1 - y2;
    return sqrt(dx * dx + dy * dy);
}

static double computeDistance(const Path& path,
                              const size_t& begin_idx,
                              const size_t& end_idx)
{
	assert(begin_idx >= 0 && begin_idx <= path.final_index);
	assert(end_idx >= 0 && end_idx <= path.final_index);

    double dx = path.points[begin_idx].x - path.points[end_idx].x;
    double dy = path.points[begin_idx].y - path.points[end_idx].y;
    return sqrt(dx * dx + dy * dy);
}

static double computeDistance(const Point& point1,
                              const Point& point2)
{
	double dx = point1.x - point2.x;
	double dy = point1.y - point2.y;
	return sqrt(dx * dx + dy * dy);
}

static void transform2DPoint(double& x,
                             double& y,
							 const double& phi,
							 const double& x0,
							 const double& y0)
{
	double temp_x = x;
	double temp_y = y;
	x = temp_x * cos(phi) - temp_y * sin(phi);
	y = temp_x * sin(phi) + temp_y * cos(phi);
	x += x0;
	y += y0;
}

static void transform2DPoints(double xs[4],
                              double ys[4],
							  const double& phi,
							  const double& x0,
							  const double& y0) // 数组作形参将自动转换为指针
{
    for(int i = 0; i < 4; i++)
    {
        double temp_x = xs[i];
        double temp_y = ys[i];
        xs[i] = temp_x * cos(phi) - temp_y * sin(phi);
        ys[i] = temp_x * sin(phi) + temp_y * cos(phi);
		xs[i] += x0;
		ys[i] += y0;
    }
}

/*@brief 在目标路径path中查找距离pose最近的点，返回该点的索引值，遍历搜索，
         滤除与当前航向偏差大于45度的点（如果路径发生交叉，yaw值之差不应小于45度），
		 当未查找到最近点或最近点过远时返回路径最大索引值
 *@param path 目标路径
 *@param pose 目标点
 #@param max_match_dis 容许距离误差
*/
static size_t findNearestPointInPath(const Path& path,
                                     const Pose& pose,
                                     const double& max_match_dis)
{
	size_t idx = 0;
	double min_dis = DBL_MAX;
	
	for(size_t i = 0; i < path.final_index; i++)
	{
		double yaw_err = path.points[i].yaw - pose.yaw;
		if(yaw_err > M_PI) yaw_err -= 2 * M_PI;
		else if(yaw_err < -M_PI) yaw_err += 2 * M_PI;
		
		if(fabs(yaw_err) > M_PI / 4) continue;
		
		double dis = computeDistance(path.points[i].x, path.points[i].y, pose.x, pose.y);
		if(dis < min_dis)
		{
			min_dis = dis;
			idx = i;
		}
	}
	if(min_dis > max_match_dis)
	{
		ROS_ERROR("[findNearestPoint] pose.x:%.2f\t pose.x:%.2f", pose.x, pose.y);
		ROS_ERROR("[findNearestPoint] cannot find correct nearest point! the nearest point distance is:%.2f", min_dis);
		return path.final_index;
	}
	return idx;
}

/*@brief 在目标路径path中查找距离(x, y)最近的点，返回该点的索引值，遍历搜索
 *@param path 目标路径
 *@param x 目标点X坐标
 *@param y 目标点Y坐标
 *@param begin_idx 搜索起点索引
 *@param end_idx 搜索终点索引
*/
static size_t findNearestPointInPath(const Path& path,
                                     const double& x,
                                     const double& y,
                                     const size_t& begin_idx,
                                     const size_t& end_idx)
{
	assert(begin_idx >= 0 && end_idx <= path.final_index);
    assert(begin_idx < end_idx);

    size_t idx;
	double min_dis = DBL_MAX;
	
	for(size_t i = begin_idx; i < end_idx; i++)
	{
		double dis = computeDistance(path.points[i].x, path.points[i].y, x, y);
		if(dis < min_dis)
		{
			min_dis = dis;
			idx = i;
		}
	}
	return idx;
}

/*@brief 查找点(x, y)到路径path的最短距离，遍历搜索
 *@param path 目标路径
 *@param x 目标点X坐标
 *@param y 目标点Y坐标
 *@param begin_idx 搜索起点索引
 *@param end_idx 搜索终点索引
*/
static double findMinDistance2Path(const Path& path,
                                   const double& x,
                                   const double& y,
                                   const size_t& begin_idx,
                                   const size_t& end_idx)
{
    assert(begin_idx >= 0 && end_idx <= path.final_index);
    assert(begin_idx < end_idx);

    size_t idx = findNearestPointInPath(path, x, y, begin_idx, end_idx);
   
    double dx = path.points[idx].x - x;
    double dy = path.points[idx].y - y;
    return sqrt(dx * dx + dy * dy);
}

/*@brief 在目标路径path中查找距离(x, y)最近的点，返回该点的索引值，根据参考点搜索
 *@param path 路径
 *@param x 目标点X坐标
 *@param y 目标点Y坐标
 *@param ref_idx 参考点索引，以此参考点展开搜索，加速计算
 *@param min_idx 最小搜索索引
 *@param max_idx 最大搜索索引
 */
static size_t findNearestPointInPath(const Path& path, 
                                     const double& x,
                                     const double& y,
						             const size_t& ref_idx,
                                     const size_t& min_idx,
						             const size_t& max_idx)
{
    assert(ref_idx >= min_idx && ref_idx <= max_idx);
    assert(min_idx >= 0 && max_idx <= path.final_index);
    assert(max_idx - min_idx >= 2);

    // 搜索方向：-1 向后搜索，1 向前搜索，0 搜索完毕
    int search_direction;
    size_t idx = ref_idx;

	if(idx == min_idx)
	{
		idx = min_idx + 1;
		search_direction = 1;
	}
	else if(idx == max_idx)
	{
		idx = max_idx - 1;
		search_direction = -1;
	}
	else
	{
		float dis2ref = pow(path.points[idx].x - x, 2) + pow(path.points[idx].y - y, 2);
		float dis2last = pow(path.points[idx - 1].x - x, 2) + pow(path.points[idx - 1].y - y, 2);
		float dis2next = pow(path.points[idx + 1].x - x, 2) + pow(path.points[idx + 1].y - y, 2);
		
        if(dis2next >= dis2ref && dis2last >= dis2ref)
        {
            search_direction = 0;
        }
        else
        {
            if(dis2next > dis2last) search_direction = -1;
            else search_direction = 1;
        }
	}
	
	while(idx > min_idx && idx < max_idx)
	{
		float dis2ref = pow(path.points[idx].x - x, 2) + pow(path.points[idx].y - y, 2);
		float dis2last = pow(path.points[idx - 1].x - x, 2) + pow(path.points[idx - 1].y - y, 2);
		float dis2next = pow(path.points[idx + 1].x - x, 2) + pow(path.points[idx + 1].y - y, 2);
		
        if((search_direction == 1 && dis2next > dis2ref) ||
		   (search_direction == -1 && dis2last > dis2ref) ||
		   (search_direction == 0))
			break;
        idx += search_direction;
	}

	return idx;
}

/*@brief 查找点(x, y)到路径path的最短距离，根据参考点搜索
 *@param path 路径
 *@param x 目标点X坐标
 *@param y 目标点Y坐标
 *@param ref_idx 参考点索引，以此参考点展开搜索，加速计算
 *@param min_idx 最小搜索索引
 *@param max_idx 最大搜索索引
 */
static double findMinDistance2Path(const Path& path, 
                                   const double& x,
                                   const double& y,
						           const size_t& ref_idx,
                                   const size_t& min_idx,
						           const size_t& max_idx)
{
    assert(ref_idx >= min_idx && ref_idx <= max_idx);
    assert(min_idx >= 0 && max_idx <= path.final_index);
    assert(max_idx - min_idx >= 2);
	
    double ref_dx = path.points[ref_idx].x - x;
    double ref_dy = path.points[ref_idx].y - y;
    double ref_dis = sqrt(ref_dx * ref_dx + ref_dy * ref_dy);

    size_t idx = findNearestPointInPath(path, x, y, ref_idx, min_idx, max_idx);

    double dx = path.points[idx].x - x;
    double dy = path.points[idx].y - y;
    double dis = sqrt(dx * dx + dy * dy);

	return dis < ref_dis ? dis : ref_dis;
}

/*@brief 计算到当前点指定距离的路径点的索引，当路径中的点无法满足条件时，返回终点索引
 *@param path 路径
 *@param expect_dis 期望距离
 *@param begin_idx 搜索起点（当前点）索引
 */
static size_t findPointInPath(const Path& path, 
                              const double& expect_dis,
						      const size_t& begin_idx)
{
    assert(expect_dis >= 0);
    assert(begin_idx >= 0 && begin_idx <= path.final_index - 1);
    
    size_t idx = begin_idx;
    double dis_sum = 0;

    while(dis_sum < expect_dis && idx < path.final_index)
    {
        double dx = path.points[idx].x - path.points[idx + 1].x;
        double dy = path.points[idx].y - path.points[idx + 1].y;
        double dis = sqrt(dx * dx + dy * dy);
		dis_sum += dis;
		idx += 1;
    }

	return idx;
}

static GpsPoint computePointOffset(const GpsPoint& point,
                                   const float& offset)
{
	GpsPoint result = point;

	// 车辆前进时，左负（offset应小于0）右正（offset应大于0）
	result.x =  offset * sin(point.yaw) + point.x;
	result.y = -offset * cos(point.yaw) + point.y;

	return result;
}

static std::pair<float, float> computeDisAndYaw(const Pose& point1,
                                                const Pose& point2)
{
	float dx = point1.x - point2.x;
	float dy = point1.y - point2.y;
	
	std::pair<float, float> dis_yaw;
	dis_yaw.first = sqrt(dx * dx + dy * dy);
	dis_yaw.second = atan2(dy, dx);
	
	if(dis_yaw.second < 0)
		dis_yaw.second += 2 * M_PI;
	return dis_yaw;
}

#endif
