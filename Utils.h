#pragma once
#include "./Restrict.h"

class Utils {
public:
	static vector<vector<double>> getMatrFromConditions(vector<RestrictBase*>& s_conditions) {
        vector<vector<double>> result;
        for (auto s : s_conditions) {
            result.push_back(s->row_matr);
        }
        return result;
    }

    static void invertGoal(vector<double>& goal) {
        for (double& g : goal) {
            g *= -1;
        }
    }

    static vector<vector<double>> transposeMatrix(vector<vector<double>>&& matr) {
        vector<vector<double>> res(matr[0].size(), vector<double>(matr.size(), 0));
        for (int i = 0; i < matr.size(); i++) {
            for (int j = 0; j < matr[0].size(); j++) {
                res[j][i] = matr[i][j];
            }
        }
        return res;
    }
};