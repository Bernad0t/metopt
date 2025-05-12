#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <queue>
#include <memory>
#include <random>
#include "./Tree.h"

class BaseLOF {
protected:
    struct Point {
        std::vector<double> features;

        Point(const std::vector<double>& f) : features(f) {}
    };

    std::vector<Point> data;
    size_t k;

    double euclideanDistance(const Point& a, const Point& b) const {
        double sum = 0.0;
        for (size_t i = 0; i < a.features.size(); ++i) {
            sum += std::pow(a.features[i] - b.features[i], 2);
        }
        return std::sqrt(sum);
    }

    virtual std::vector<size_t> findKNeighbors(size_t pointIdx) = 0;

    virtual double reachabilityDistance(size_t aIdx, size_t bIdx) = 0;
    virtual double localReachabilityDensity(size_t pointIdx) = 0;

public:
    virtual double computeLOF(size_t pointIdx) = 0;
    std::vector<double> computeAllLOFs() {
        std::vector<double> lofScores(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            lofScores[i] = computeLOF(i);
        }
        return lofScores;
    }
};

class FastLOF : public BaseLOF {
private:
    std::unique_ptr<SimpleKDTree> kd_tree;


    std::vector<size_t> findKNeighbors(size_t pointIdx) override {
        auto indices = kd_tree->search(data[pointIdx].features, k + 1); // k+1 чтобы включить саму точку

        // ѕропускаем первую точку (это сама точка)
        std::vector<size_t> neighbors;
        for (size_t i = 1; i < indices.size(); ++i) {
            neighbors.push_back(static_cast<size_t>(indices[i]));
        }
        return neighbors;
    }

    double reachabilityDistance(size_t aIdx, size_t bIdx) override {
        double distAB = euclideanDistance(data[aIdx], data[bIdx]);
        auto neighborsB = findKNeighbors(bIdx);
        double kDistanceB = neighborsB.empty() ? 0.0 : 0.0;

        if (!neighborsB.empty()) {
            double max_dist = 0.0;
            for (auto neighbor : neighborsB) {
                double dist = euclideanDistance(data[bIdx], data[neighbor]);
                if (dist > max_dist) max_dist = dist;
            }
            kDistanceB = max_dist;
        }

        return std::max(distAB, kDistanceB);
    }

    double localReachabilityDensity(size_t pointIdx) override {
        auto neighbors = findKNeighbors(pointIdx);
        double sumReachDist = 0.0;

        for (size_t neighborIdx : neighbors) {
            sumReachDist += reachabilityDistance(pointIdx, neighborIdx);
        }

        return sumReachDist == 0.0 ?
            std::numeric_limits<double>::max() :
            neighbors.size() / sumReachDist;
    }

public:
    FastLOF(const std::vector<std::vector<double>>& inputData, size_t neighborsCount) {
        k = neighborsCount;
        for (const auto& point : inputData) {
            data.emplace_back(point);
        }

        // —троим KD-дерево дл€ приближенного поиска соседей
        std::vector<std::vector<double>> points;
        for (const auto& p : data) {
            points.push_back(p.features);
        }
        kd_tree = std::make_unique<SimpleKDTree>(points);
    }

    double computeLOF(size_t pointIdx) override {
        double lrdP = localReachabilityDensity(pointIdx);
        auto neighbors = findKNeighbors(pointIdx);

        double sumLRD = 0.0;
        for (size_t neighborIdx : neighbors) {
            sumLRD += localReachabilityDensity(neighborIdx);
        }

        return (sumLRD / neighbors.size()) / lrdP;
    }
};

class LOF : public BaseLOF{
private:
    std::vector<size_t> findKNeighbors(size_t pointIdx) override {
        std::priority_queue<std::pair<double, size_t>> maxHeap;

        for (size_t i = 0; i < data.size(); ++i) {
            if (i == pointIdx) continue;

            double dist = euclideanDistance(data[pointIdx], data[i]);
            maxHeap.push({ dist, i });

            if (maxHeap.size() > k) {
                maxHeap.pop();
            }
        }

        std::vector<size_t> neighbors;
        while (!maxHeap.empty()) {
            neighbors.push_back(maxHeap.top().second);
            maxHeap.pop();
        }
        return neighbors;
    }

    double reachabilityDistance(size_t aIdx, size_t bIdx) override {
        double distAB = euclideanDistance(data[aIdx], data[bIdx]);
        auto neighborsB = findKNeighbors(bIdx);
        double kDistanceB = neighborsB.empty() ? 0.0 :
            euclideanDistance(data[bIdx], data[neighborsB.back()]);

        return std::max(distAB, kDistanceB);
    }

    double localReachabilityDensity(size_t pointIdx) override {
        auto neighbors = findKNeighbors(pointIdx);
        double sumReachDist = 0.0;

        for (size_t neighborIdx : neighbors) {
            sumReachDist += reachabilityDistance(pointIdx, neighborIdx);
        }

        return sumReachDist == 0.0 ?
            std::numeric_limits<double>::max() :
            neighbors.size() / sumReachDist;
    }

public:
    LOF(const std::vector<std::vector<double>>& inputData, size_t neighborsCount){
        k = neighborsCount;
        for (const auto& point : inputData) {
            data.emplace_back(point);
        }
    }

    double computeLOF(size_t pointIdx) override {
        double lrdP = localReachabilityDensity(pointIdx);
        auto neighbors = findKNeighbors(pointIdx);

        double sumLRD = 0.0;
        for (size_t neighborIdx : neighbors) {
            sumLRD += localReachabilityDensity(neighborIdx);
        }

        return (sumLRD / neighbors.size()) / lrdP;
    }
};