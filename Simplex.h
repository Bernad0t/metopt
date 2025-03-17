#pragma once
#include "./OrganizedToCanon.h"
#include <iostream>
#include <vector>
#include <iomanip>

class Simplex {
	vector<vector<double>> table; // последний парметр - свободный член, последняя строка - целевая функция
	vector<int> basis;

	int searchMainRow(int pivotCol) {
		int pivotRow = -1;
		double minRatio = 0;
		for (int i = 0; i < table.size(); ++i) {
			if (table[i][pivotCol] > 0) {
				double ratio = table[i].back() / table[i][pivotCol];
				if (ratio <= minRatio || minRatio == 0) {
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

	void updateBasis() {
		for (int column = 0; column < table[0].size(); column++) {
			int nonZero = 0;
			int index;
			for (int row = 0; row < table.size(); row++) {
				if (table[row][column] != 0) {
					nonZero++;
					index = row;
				}
				if (nonZero > 1) {
					break;
				}
			}
			if (nonZero == 1)
				basis[index] = column;
		}
	}

	bool solveStep() {
		// Найти ведущий столбец (с отрицательным коэффициентом в целевой строке)
		int pivotCol = -1;
		double min;
		for (int j = 0; j < table[0].size() - 1; ++j) {
			if (table[table.size() - 1][j] < 0 && (pivotCol == -1 || table[table.size() - 1][j] <= min)) {
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
		updateBasis();
		return true;
	}

	void printTable() const {
		cout << "\nSimplex Table:\n";

		for (const auto& row : table) {
			for (double value : row) {
				cout << setw(10) << fixed << setprecision(6) << value << " ";
			}
			cout << endl;
		}
	}
public:
	Simplex(OrganizedToCanon* task) {
		basis.resize(task->s_conditions.size(), -1);
		int countBasis = 0;
		for (auto cond : task->s_conditions) {
			if (cond->getType() != Type::eqw) {
				basis[countBasis] = task->goal.size() + countBasis; // номер слековой переменной
				countBasis++;
			}
		}
		task->createCanonForm(); // задача теперь на max
		task->print();
		for (auto cond : task->s_conditions) {
			vector<double> row = cond->row_matr;
			row.push_back(cond->param);
			table.push_back(row);
		}
		vector<double> goal = task->goal;
		Utils::invertGoal(goal);
		goal.push_back(0);
		table.push_back(goal);
	}

	double solve() {
		printTable();
		while (solveStep()) {
			printTable();
			cout << "BASIS: ";
			for (auto bas : basis) {
				cout << bas << ' ';
			}
			cout << endl;
		}
		return table.back().back();
	}
};