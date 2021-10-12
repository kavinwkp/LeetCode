#pragma once

#include <vector>
#include <string>

struct Result
{
	std::vector<std::vector<int>> path;
	std::vector<double> load;
	std::vector<double> mileage;
	int numGeneration;
	std::string toString() const;

	double totalTime() const;		// 总配送时间
	double totalMileage() const;	// 总里程
	int totalCarUse() const;		// 使用的车辆数
};