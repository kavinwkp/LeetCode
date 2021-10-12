#pragma once

#include "Node.h"
#include "Car.h"
#include "Result.h"

class VRP
{
	friend class Chrom;
public:
	VRP();
	void readDataFromFile(const std::string& filename);
	void addNode(double x, double y, double demand);
	void addCar(double capacity, double disLimit);
	void setWeights(double k1, double k2, double k3);
	std::string toString() const;
	void solve(Result& res, int, int);

private:
	int cNode, cCar;	// 存储节点数量和卡车数量
	std::vector<Node> nodeInfo;
	std::vector<Car> carInfo;
	std::vector<std::vector<double>> dis;
	double k1, k2, k3;
};