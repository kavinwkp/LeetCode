#pragma once

#include "Node.h"
#include "Car.h"
#include "Result.h"
#include "VRP.h"
class Chrom
{
	friend class VRP;
public:
	Chrom(const VRP& vrp);
	void update();
	void mutation();		// 突变
	double fitness() const;
	bool operator<(const Chrom& c) const;
	Chrom& operator=(const Chrom& c);
	void decode(Result& res) const;
	std::string toString() const;

private:
	const std::vector<Node>& nodeInfo;
	const std::vector<Car>& carInfo;
	const std::vector<std::vector<double>>& dis;
	double k1, k2, k3;

	std::vector<int> gene;
	std::vector<double> mileage, load;	// 每辆车的行驶距离和载荷
	double time, length;
	int cnt;	// 最后实际使用的车辆数
	bool valid;
};