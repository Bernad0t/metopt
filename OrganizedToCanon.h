#pragma once
#include "./Restrict.h"
#include <iostream>
#include "./Utils.h"
#include "Task.h"

class Simplex;

class OrganizedToCanon: public TaskBase {
protected:
    friend class Simplex;

    RestrictBase* fromXcond(RestrictBase* x_restrict, vector<double> row, double param) {
        switch (x_restrict->getType())
        {
        case (Type::big):
            return new LowerStrongRestrict(move(row), param);
            break;
        case (Type::low):
            return new BiggerStrongRestrict(move(row), param);
            break;
        case (Type::eqw):
            return nullptr;
            break;
        default:
            break;
        }
    }

    RestrictBase* fromScond(RestrictBase* s_restrict, vector<double> row) {
        switch (s_restrict->getType())
        {
        case (Type::low):
            return new LowerStrongRestrict(move(row), 0);
            break;
        case (Type::big):
            return new BiggerStrongRestrict(move(row), 0);
            break;
        case (Type::eqw):
            return nullptr;
            break;
        default:
            break;
        }
    }

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

    void doParamsNatural() { // bi >= 0
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
            row_matr.push_back(-row_matr[index]);
        };
        for (int row = 0; row < s_conditions.size(); row++) {
            decomposeX(index, s_conditions[row]->row_matr);
        }

        
        decomposeX(index, goal);
    }

    void transformMatrixByNewX(int posUpdatedX) { // учесть изменение икса
        int index = x_conditions[posUpdatedX]->getIndexNotZeroValue();
        double mult = x_conditions[posUpdatedX]->row_matr[index];
        for (auto& cond : s_conditions) {
            cond->row_matr[index] *= mult;
        }
        goal[index] *= mult;
    }

    void makeUnknownNatural() {
        auto findNotBigger = [this]() {
            for (auto cond : x_conditions)
                if (cond->getType() != Type::big)
                    return true;
            return false;
        };
        if (goal.size() != x_conditions.size() || findNotBigger()) { // хотя бы 1 переменная любая
            int number_unknown = x_conditions.size() + 2 * (goal.size() - x_conditions.size()); // в x_conditions >= 0, а каждый отсутствующий там даст по 2 новых параметра
            vector<RestrictBase*> new_x_conditions = vector<RestrictBase*>(number_unknown, nullptr);
            int cursor_old = 0;
            int added_params = 0;
            bool should_change_conditions = false;
            for (int i = 0; i + added_params< new_x_conditions.size(); i++) {
                if (cursor_old >= x_conditions.size() || x_conditions[cursor_old]->getIndexNotZeroValue() != i) {
                    RestrictBase* new_x = new BiggerStrongRestrict(move(vector<double>(number_unknown, 0)), 0);
                    new_x->row_matr[i] = 1;
                    new_x_conditions[x_conditions.size() + added_params] = new_x;
                    decompose(i);
                    added_params++;
                    RestrictBase* new_x_second = new BiggerStrongRestrict(move(vector<double>(number_unknown, 0)), 0);
                    new_x_second->row_matr[goal.size() - 1] = 1;
                    new_x_conditions[x_conditions.size() + added_params] = new_x_second;
                    added_params++;
                }
                else {
                    x_conditions[cursor_old]->row_matr.resize(number_unknown, 0);
                    RestrictBase* tmp = x_conditions[cursor_old]->toBigger();
                    if (tmp != x_conditions[cursor_old]) {
                        delete x_conditions[cursor_old];
                        x_conditions[cursor_old] = tmp;
                        transformMatrixByNewX(cursor_old);
                    }
                    new_x_conditions[cursor_old] = tmp;
                    cursor_old++;
                }
            }
            x_conditions = new_x_conditions; // delete делать не надо, т к скопировано в new_x_conditions
        }
    }

public:
    using TaskBase::TaskBase;
    OrganizedToCanon* goToAgainst() {
        vector<vector<double>> transp_matr = Utils::transposeMatrix(Utils::getMatrFromConditions(s_conditions));
        vector<RestrictBase*> s_dual_conditions;
        vector<RestrictBase*> x_dual_conditions;
        for (int i = 0; i < x_conditions.size(); i++) {
            if (x_conditions[i]->getIndexNotZeroValue() != i) {
                for (int j = i; j < x_conditions[i]->getIndexNotZeroValue(); j++)
                    s_dual_conditions.push_back(new EqwRestrict(move(transp_matr[j]), goal[j])); // целевая функция - новые bi
            }
            RestrictBase* restrict = fromXcond(x_conditions[i], transp_matr[i], goal[i]);
            s_dual_conditions.push_back(restrict); // целевая функция - новые bi
        }
        for (int i = x_conditions.back()->getIndexNotZeroValue() + 1; i < goal.size(); i++) {
            s_dual_conditions.push_back(new EqwRestrict(move(transp_matr[i]), goal[i]));
        }
        vector<double> new_goal;
        for (int i = 0; i < s_conditions.size(); i++) {
            new_goal.push_back(s_conditions[i]->param);
        }
        for (int i = 0; i < new_goal.size(); i++) {
            vector<double> x(new_goal.size(), 0);
            x[i] = 1;
            RestrictBase* restrict = fromScond(s_conditions[i], x);
            if (restrict != nullptr)
                x_dual_conditions.push_back(restrict);
        }
        return new OrganizedToCanon(s_dual_conditions, x_dual_conditions, new_goal, !isMinTask);
    }

    void createCanonForm() {
        if (!isFullRank()) {
            throw runtime_error("Матрица не имеет полный ранг. Задача не может быть преобразована в канонический вид.");
        }
        if (isMinTask)
            Utils::invertGoal(goal);
        doParamsNatural();
        makeUnknownNatural();
        /*cout << "after naturale" << endl;
        print();*/
        for (auto& condition : s_conditions) {
            EqwRestrict* eqw = condition->toEqw();
            if (condition != eqw) {
                delete condition;
                condition = eqw;
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
        }

        for (int i = goal.size(); i < x_conditions.size(); i++) {
            goal.push_back(0);
        }
    }
};