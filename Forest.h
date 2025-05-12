#pragma once

#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>

class IsolationForest {
private:
    struct ITree {
        struct Node {
            std::unique_ptr<Node> left;
            std::unique_ptr<Node> right;
            double split_value;
            int split_dim;
            bool is_leaf;
            size_t size;

            Node() : is_leaf(false), size(0), split_value(0), split_dim(0) {}
        };

        std::unique_ptr<Node> root;
        size_t max_depth;

        ITree(size_t depth) : max_depth(depth) {}

        void build(std::unique_ptr<Node>& node, std::vector<std::vector<double>>& data,
            size_t depth, std::mt19937& rng) {
            if (data.empty()) {
                node.reset();
                return;
            }

            // Условия остановки
            if (depth >= max_depth || data.size() <= 1) {
                node = std::make_unique<Node>();
                node->is_leaf = true;
                node->size = data.size();
                return;
            }

            // Выбираем случайный признак
            std::uniform_int_distribution<size_t> dim_dist(0, data[0].size() - 1);
            size_t split_dim = dim_dist(rng);

            // Находим min и max
            auto minmax = std::minmax_element(data.begin(), data.end(),
                [split_dim](const auto& a, const auto& b) {
                    return a[split_dim] < b[split_dim];
                });

            double min_val = (*minmax.first)[split_dim];
            double max_val = (*minmax.second)[split_dim];

            // Случайное значение разбиения
            std::uniform_real_distribution<double> val_dist(min_val, max_val);
            double split_value = val_dist(rng);

            // Разделяем данные
            std::vector<std::vector<double>> left_data, right_data;
            for (auto& point : data) {
                if (point[split_dim] < split_value) {
                    left_data.push_back(std::move(point));
                }
                else {
                    right_data.push_back(std::move(point));
                }
            }

            // Проверка на неудачное разделение
            if (left_data.empty() || right_data.empty()) {
                node = std::make_unique<Node>();
                node->is_leaf = true;
                node->size = data.size();
                return;
            }

            // Создаем внутренний узел
            node = std::make_unique<Node>();
            node->split_dim = split_dim;
            node->split_value = split_value;
            node->size = data.size();

            // Рекурсивное построение
            build(node->left, left_data, depth + 1, rng);
            build(node->right, right_data, depth + 1, rng);
        }

        double pathLength(const std::vector<double>& point, const Node* node, double current_length) const {
            if (!node) return current_length;

            if (node->is_leaf) {
                return current_length + (node->size > 1 ? 2 * (log(node->size) + 0.5772156649) : 1);
            }

            if (point[node->split_dim] < node->split_value) {
                return pathLength(point, node->left.get(), current_length + 1);
            }
            return pathLength(point, node->right.get(), current_length + 1);
        }
    };

    std::vector<std::unique_ptr<ITree>> trees;
    size_t num_trees;
    size_t max_samples;
    size_t max_depth;

    double c(size_t n) const {
        if (n <= 1) return 1;
        return 2 * (log(n - 1) + 0.5772156649) - 2 * (n - 1) / n;
    }

public:
    IsolationForest(size_t n_trees = 100, size_t m_samples = 256, size_t m_depth = 8)
        : num_trees(n_trees), max_samples(m_samples), max_depth(m_depth) {}

    ~IsolationForest() {
        trees.clear();
    }

    void fit(const std::vector<std::vector<double>>& data) {
        if (data.empty() || data[0].empty()) return;

        trees.clear();
        std::random_device rd;
        std::mt19937 rng(rd());

        size_t actual_samples = std::min(max_samples, data.size());

        for (size_t i = 0; i < num_trees; ++i) {
            // Создаем копию данных для подвыборки
            std::vector<std::vector<double>> data_copy = data;

            // Выбираем случайную подвыборку
            std::shuffle(data_copy.begin(), data_copy.end(), rng);
            std::vector<std::vector<double>> subsample(
                data_copy.begin(),
                data_copy.begin() + actual_samples);

            auto tree = std::make_unique<ITree>(max_depth);
            tree->build(tree->root, subsample, 0, rng);
            trees.push_back(std::move(tree));
        }
    }

    std::vector<double> anomalyScores(const std::vector<std::vector<double>>& points) const {
        std::vector<double> scores;
        scores.reserve(points.size());

        for (const auto& point : points) {
            double avg_path = 0.0;
            for (const auto& tree : trees) {
                if (tree && tree->root) {
                    avg_path += tree->pathLength(point, tree->root.get(), 0);
                }
            }

            if (!trees.empty()) {
                double score = pow(2, -avg_path / trees.size() / c(max_samples));
                scores.push_back(score);
            }
            else {
                scores.push_back(0.0);
            }
        }

        // Нормализация оценок
        if (!scores.empty()) {
            double max_score = *std::max_element(scores.begin(), scores.end());
            if (max_score > 0) {
                for (auto& score : scores) {
                    score /= max_score;
                }
            }
        }

        return scores;
    }
};