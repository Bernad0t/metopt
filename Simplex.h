#pragma once
#include "./OrganizedToCanon.h"

class Simplex {
	vector<vector<double>> table; // последний парметр - свободный член, последняя строка - целевая функция

	int searchMainRow(int pivotCol) {
		int pivotRow = -1;
		double minRatio = table[0].back();
		for (int i = 0; i < table.size(); ++i) {
			if (table[i][pivotCol] > 0) {
				double ratio = table[i].back() / table[i][pivotCol];
				if (ratio <= minRatio) {
					minRatio = ratio;
					pivotRow = i;
				}
			}
		}
		if (pivotRow == -1) {
			throw runtime_error("Task is unbounded."); // Целевая функция не ограничена
		}
		return pivotRow;
	}

	bool solveStep() {
		// Найти ведущий столбец (с отрицательным коэффициентом в целевой строке)
		int pivotCol = -1;
		double min = table.back()[0];
		for (int j = 0; j < table[0].size() - 1; ++j) {
			if (table[table.size() - 1][j] < 0 && table[table.size() - 1][j] <= min) {
				pivotCol = j;
				min = table[table.size() - 1][j];
			}
		}
		if (pivotCol == -1) {
			return false; // Оптимальное решение найдено
		}
		int pivotRow = searchMainRow(pivotCol);

		// Приведение ведущего элемента к 1 и остальных элементов столбца к 0
		double pivotValue = table[pivotRow][pivotCol];
		for (int j = 0; j < table[0].size(); ++j) {
			table[pivotRow][j] /= pivotValue;
		}

		for (int i = 0; i < table.size(); ++i) {
			if (i != pivotRow) {
				double factor = table[i][pivotCol];
				for (int j = 0; j < table[0].size(); ++j) {
					table[i][j] -= factor * table[pivotRow][j];
				}
			}
		}
		return true;
	}

	void printTable() const {
		cout << "\nSimplex Table:" << endl;
		for (const auto& row : table) {
			for (double value : row) {
				cout << value << " ";
			}
			cout << endl;
		}
	}
public:
	Simplex(OrganizedToCanon* task) {
		task->createCanonForm();
		task->print();
		for (auto cond : task->s_conditions) {
			vector<double> row = cond->row_matr;
			row.push_back(cond->param);
			table.push_back(row);
		}
		vector<double> goal = task->goal;
		//Utils::invertGoal(goal);
		goal.push_back(0);
		table.push_back(goal);
	}

	double solve() {
		while (solveStep()) {
			printTable();
		}
		return table.back().back();
	}
};