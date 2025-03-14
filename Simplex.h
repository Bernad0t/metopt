#pragma once
#include "./DualTask.h"

class Simplex {
	DualTask* task; 
	vector<vector<double>> table; // последний парметр - свободный член, последн€€ строка - целева€ функци€

	int searchMainRow(int j) { // индекс главного столбца пришел и сразу делим на глав столбец
		double min = table[0][table[0].size() - 1];
		int indexRow ;
		for (int i = 0; i < table.size(); i++) {
			if (table[i][table[0].size() - 1] != 0 && table[i][j] != 0 && table[i][table[0].size() - 1] > 0) {
				table[i][table[0].size() - 1] /= table[i][j];
				if (table[i][table[0].size() - 1] < min) {
					min = table[i][table[0].size() - 1];
					indexRow = i;
				}
			}
		} 
		return indexRow;
	}

	double calculateMeans(int rowInd, int colInd, int i, int j) { // int rowInd, int colInd главные
		return table[i][j] - table[i][colInd] * table[rowInd][j] / table[rowInd][colInd];
	}

	void transform(int rowInd, int colInd) { // не учел свободные члены TODO
		for (int i = 0; i < table[0].size(); i++) { // главную строку мен€ем
			if (i == colInd)
				continue;
			table[rowInd][i] /= table[rowInd][colInd];
		}
		for (int i = 0; i < table.size(); i++) { // главный столбец мен€ем
			if (i == rowInd)
				continue;
			table[i][colInd] /= -table[rowInd][colInd];
		}
		for (int row = 0; row < table.size(); row++){ // мен€ем в матрице на новые bij
			if (row == rowInd)
				continue;
			for (int col = 0; col < table[0].size(); col++) {
				if (col == colInd)
					continue;
				table[row][col] = calculateMeans(rowInd, colInd, row, col);
			}
		}
	}
public:
	Simplex(DualTask* task) {
		this->task = task;
		task->invertGoal();
		for (auto cond : task->s_conditions) {
			vector<double> row = cond->row_matr;
			row.push_back(cond->param);
			table.push_back(row);
		}
		vector<double> goal = task->goal;
		goal.push_back(0);
		table.push_back(goal);
	}

	void solve() {
		vector<double> goalTemp = task->goal;

	}
};