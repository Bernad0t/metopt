#pragma once
#include "./Restrict.h"
#include <iostream>
#include "./Utils.h"
#include "Task.h"

class Simplex;

class OrganizedToCanon: public TaskBase {
protected:
    friend class Simplex;
    int rowReduction(vector<vector<double>>& mat) {
        int n = mat.size();    // Число строк
        int m = mat[0].size(); // Число столбцов
        int rank = 0;

        for (int col = 0; col < m; col++) {
            // Ищем ненулевой элемент в текущем столбце
            int pivot = rank;
            while (pivot < n && mat[pivot][col] == 0) {
                pivot++;
            }

            // Если найден ненулевой элемент, обменяем строки
            if (pivot != n) {
                swap(mat[rank], mat[pivot]);

                // Привидение текущей строки к стандартному виду: нормируем
                for (int c = m - 1; c >= col; --c) {
                    mat[rank][c] /= mat[rank][col];
                }

                // Приведение элементов в столбце ниже текущей строки к нулю
                for (int row = 0; row < n; row++) {
                    if (row != rank && mat[row][col] != 0) {
                        int factor = mat[row][col];
                        for (int c = col; c < m; c++) {
                            mat[row][c] -= factor * mat[rank][c];
                        }
                    }
                }
                rank++;
            }
        }

        return rank;
    }

    // Функция для проверки матрицы на полный ранг
    bool isFullRank() {
        vector<vector<double>> temp = Utils::getMatrFromConditions(s_conditions); // Создаем копию матрицы для изменения
        int n = temp.size();
        int m = temp[0].size();
        int rank = rowReduction(temp);
        return rank == min(n, m);
    }

    void doParamsNatural() {
        for (int i = 0; i < s_conditions.size(); i++) {
            if (s_conditions[i]->param < 0) {
                RestrictBase* tmp = s_conditions[i];
                s_conditions[i] = s_conditions[i]->multiplyOnNumber(-1);
                delete tmp;
            }
        }
    }

    void decompose(int index) {
        auto decomposeX = [](int index, vector<double>& row_matr) {
            vector<double> new_row_matr = vector<double>(row_matr.size() + 1, 0);
            int isAdded = 0;
            for (int i = 0; i < new_row_matr.size(); i++) {
                if (i == index) {
                    isAdded++;
                    new_row_matr[i] = row_matr[i];
                    new_row_matr[i + 1] = -row_matr[i];
                    i += 2;
                    continue;
                }
                new_row_matr[i] = row_matr[i - isAdded];
            }
            row_matr = new_row_matr;
        };
        for (int row = 0; row < s_conditions.size(); row++) {
            decomposeX(index, s_conditions[row]->row_matr);
        }

        
        decomposeX(index, goal);
    }

    void makeUnknownNatural() {
        if (goal.size() != x_conditions.size()) { // хотя бы 1 переменная любая
            int number_unknown = x_conditions.size() + 2 * (goal.size() - x_conditions.size()); // в x_conditions >= 0, а каждый отсутствующий там даст по 2 новых параметра
            vector<RestrictBase*> new_x_conditions = vector<RestrictBase*>(number_unknown, nullptr);
            int cursor_old = 0;
            int added_params = 0;
            bool should_change_conditions = false;
            for (int i = 0; i < new_x_conditions.size(); i++) {
                if (cursor_old >= x_conditions.size() || x_conditions[cursor_old]->getIndexNaturalValue() != i - added_params) {
                    RestrictBase* new_x = new BiggerStrongRestrict(move(vector<double>(number_unknown, 0)), 0);
                    new_x->row_matr[i] = 1;
                    new_x_conditions[i] = new_x;
                    if (should_change_conditions) {
                        should_change_conditions = false;
                        decompose(i - added_params);
                    }
                    else {
                        should_change_conditions = true;
                        added_params++;
                    }
                }
                else {
                    x_conditions[i]->row_matr.resize(number_unknown, 0);
                    new_x_conditions[i] = x_conditions[i];
                    cursor_old++;
                }
            }
            x_conditions = new_x_conditions; // delete делать не надо, т к скопировано в new_x_conditions
        }
    }

public:
    using TaskBase::TaskBase;
    void createCanonForm() {
        if (!isFullRank()) {
            throw runtime_error("Матрица не имеет полный ранг. Задача не может быть преобразована в канонический вид.");
        }

        if (isMinTask)
            Utils::invertGoal(goal);
        doParamsNatural();
        makeUnknownNatural();

        for (auto& condition : s_conditions) {
            EqwRestrict* eqw = condition->toEqw();
            if (condition != eqw) {
                delete condition;
                condition = eqw;
            }
            for (auto& condition : s_conditions) { // новый параметр с коэф 0
                if (condition != eqw) {
                    condition->row_matr.push_back(0);
                }
            }
            for (auto& condition : x_conditions) { // новый параметр должны учесть
                condition->row_matr.push_back(0);
            }
            vector<double> new_x = vector<double>(x_conditions.size() + 1, 0);
            new_x[new_x.size() - 1] = 1;
            x_conditions.push_back(new BiggerStrongRestrict(move(new_x), 0));
        }

        for (int i = goal.size(); i < x_conditions.size(); i++) {
            goal.push_back(0);
        }
    }
};