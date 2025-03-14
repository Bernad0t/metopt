#pragma once
#include "./Restrict.h"
#include <iostream>
#include <algorithm>
#include "./OrganizedToCanon.h"
#include "./Utils.h"

//class Simplex;

class TaskBase {
protected:
	vector<RestrictBase*> s_conditions;
	vector<RestrictBase*> x_conditions; // должно быть отсортированным по xi, также должно быть >= 0
	vector<double> goal;
	bool isMinTask;

public:
	TaskBase(bool isMin) {
		isMinTask = isMin;
	}
	void addRestriction(RestrictBase* condition) {
		s_conditions.push_back(condition);
	}

	void addXCondition(RestrictBase* condition) {
		x_conditions.push_back(condition);
	}

	void setGoal(vector<double>& new_goal) {
		goal = new_goal;
	}

	void invertGoal() {
		Utils::invertGoal(goal);
	}

	void print() {
		cout << "s_cond" << endl;
		for (auto& condition : s_conditions) {
			condition->print();
		}
		cout << "\nx_cond" << endl;
		for (auto& condition : x_conditions) {
			condition->print();
		}
		cout << "\ngoal" << endl;
		for (int i = 0; i < goal.size(); i++) {
			cout << goal[i] << " ";
		}
		cout << endl;
	}

	~TaskBase() {
		for (auto* cond : s_conditions) {
			delete cond;
		}
		for (auto* cond : x_conditions) {
			delete cond;
		}
	}
};