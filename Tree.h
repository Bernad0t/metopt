#pragma once
#include "./LOF.h"
// Упрощенная реализация KD-дерева для приближенного поиска ближайших соседей
class SimpleKDTree {
private:
    struct Node {
        std::vector<double> point;
        int index;
        std::unique_ptr<Node> left;
        std::unique_ptr<Node> right;
        int axis;

        Node(const std::vector<double>& pt, int idx): point(pt), index(idx), left(nullptr), right(nullptr), axis(0) {}
    };

    std::unique_ptr<Node> root;
    size_t num_dimensions;
    size_t leaf_size;

    // Рекурсивное построение дерева
    std::unique_ptr<Node> buildTree(std::vector<std::pair<std::vector<double>, int>>& points,
        int depth, int start, int end) {
        if (start >= end) return nullptr;

        int axis = depth % num_dimensions;
        int mid = start + (end - start) / 2;

        // Сортировка по текущей оси
        std::nth_element(points.begin() + start,
            points.begin() + mid,
            points.begin() + end,
            [axis](const auto& a, const auto& b) {
                return a.first[axis] < b.first[axis];
            });

        auto node = std::make_unique<Node>(points[mid].first, points[mid].second);
        node->axis = axis;

        // Рекурсивное построение поддеревьев
        node->left = buildTree(points, depth + 1, start, mid);
        node->right = buildTree(points, depth + 1, mid + 1, end);

        return node;
    }

    // Рекурсивный поиск ближайших соседей
    void searchKNN(const Node* node, const std::vector<double>& query,
        size_t k, std::priority_queue<std::pair<double, int>>& heap) const {
        if (!node) return;

        // Вычисляем расстояние до текущей точки
        double dist = 0.0;
        for (size_t i = 0; i < num_dimensions; ++i) {
            dist += (query[i] - node->point[i]) * (query[i] - node->point[i]);
        }
        dist = std::sqrt(dist);

        // Добавляем точку в кучу
        if (heap.size() < k) {
            heap.emplace(dist, node->index);
        }
        else if (dist < heap.top().first) {
            heap.pop();
            heap.emplace(dist, node->index);
        }

        // Определяем, в каком поддереве искать
        double diff = query[node->axis] - node->point[node->axis];
        Node* first = diff < 0 ? node->left.get() : node->right.get();
        Node* second = diff < 0 ? node->right.get() : node->left.get();

        // Рекурсивный поиск в ближайшем поддереве
        searchKNN(first, query, k, heap);

        // Проверяем, нужно ли искать в дальнем поддереве
        if (heap.size() < k || diff * diff < heap.top().first) {
            searchKNN(second, query, k, heap);
        }
    }

public:
    SimpleKDTree(const std::vector<std::vector<double>>& points, size_t leaf_sz = 10)
        : num_dimensions(points.empty() ? 0 : points[0].size()), leaf_size(leaf_sz) {

        std::vector<std::pair<std::vector<double>, int>> indexed_points;
        indexed_points.reserve(points.size());

        for (int i = 0; i < points.size(); ++i) {
            indexed_points.emplace_back(points[i], i);
        }

        root = buildTree(indexed_points, 0, 0, indexed_points.size());
    }

    // Поиск k ближайших соседей
    std::vector<int> search(const std::vector<double>& query, size_t k) const {
        std::priority_queue<std::pair<double, int>> heap;
        searchKNN(root.get(), query, k, heap);

        std::vector<int> indices;
        while (!heap.empty()) {
            indices.push_back(heap.top().second);
            heap.pop();
        }

        // Индексы возвращаются в порядке убывания расстояния
        std::reverse(indices.begin(), indices.end());
        return indices;
    }
};