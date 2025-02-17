#pragma once
#include <vector>
#include <iostream>
#include <utility> // Äëÿ std::move

using namespace std;


class EqwRestrict;
class BiggerStrongRestrict;

class RestrictBase {
public:
	double param;
	vector<double> row_matr;
	RestrictBase(vector<double>&& row_matr, double param) : row_matr(std::move(row_matr)), param(param) {}
	virtual bool checkCorrect(vector<double> x) = 0;

	virtual RestrictBase* multiplyOnNumber(double number) {
		param *= number;
		for (int i = 0; i < row_matr.size(); i++) {
			row_matr[i] *= number;
		}
		return this;
	}

	virtual EqwRestrict* toEqw() = 0;

	int getIndexNaturalValue() {
		for (int i = 0; i < row_matr.size(); i++) {
			if (row_matr[i] != 0)
				return i;
		}
		return -1;
	}

	virtual void print() {
		for (int i = 0; i < row_matr.size(); i++) {
			cout << row_matr[i] << " ";
		}
	}
};

class EqwRestrict : public RestrictBase { // ==
public:
	EqwRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res != param)
			return false;
		return true;
	}

	RestrictBase* multiplyOnNumber(double number) override {
		RestrictBase* res = RestrictBase::multiplyOnNumber(number);
		return new EqwRestrict(move(res->row_matr), res->param);
	}

	EqwRestrict* toEqw() override {
		row_matr.push_back(0);
		return this;
	}

	void print() {
		RestrictBase::print();
		cout << "==" << param << endl;
	}
};

class LowerStrongRestrict : public RestrictBase { // <=
public:
	LowerStrongRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res > param)
			return false;
		return true;
	}

	RestrictBase* multiplyOnNumber(double number) override;

	EqwRestrict* toEqw() override {
		row_matr.push_back(1);
		return new EqwRestrict(move(row_matr), param);
	}

	void print() {
		RestrictBase::print();
		cout << "<=" << param << endl;
	}
};

class BiggerStrongRestrict : public RestrictBase { // >=
public:
	BiggerStrongRestrict(vector<double>&& params, double param) : RestrictBase(std::move(params), param) {}

	bool checkCorrect(vector<double> x) {
		double res = 0;
		for (int i = 0; i < x.size(); i++) {
			res += x[i] * row_matr[i];
		}
		if (res < param)
			return false;
		return true;
	}

	RestrictBase* multiplyOnNumber(double number) override {
		RestrictBase* res = RestrictBase::multiplyOnNumber(number);
		if (number > 0)
			return new BiggerStrongRestrict(move(res->row_matr), res->param);
		else
			return new LowerStrongRestrict(move(res->row_matr), res->param);
	}

	EqwRestrict* toEqw() override {
		row_matr.push_back(-1);
		return new EqwRestrict(move(row_matr), param);
	}

	void print() {
		RestrictBase::print();
		cout << ">=" << param << endl;
	}
};

RestrictBase* LowerStrongRestrict::multiplyOnNumber(double number) {
	RestrictBase* res = RestrictBase::multiplyOnNumber(number);
	if (number > 0)
		return new LowerStrongRestrict(move(res->row_matr), res->param);
	else
		return new BiggerStrongRestrict(move(res->row_matr), res->param);
}