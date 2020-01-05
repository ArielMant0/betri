#pragma once

#include <string>
#include <regex>
#include <vector>
#include <algorithm>
#include <fstream>

#include "BezierMathUtil.hh"

static void updateRaycastingFormula(int n, std::string shaderDir)
{
	std::string formula;
	std::string subFormula;
	std::string tmpFormula;

	std::string plusDelimiter = " + ";
	std::string minusDelimiter = " - ";

	std::vector<std::string> posFormulaVec;
	std::vector<std::string> negFormulaVec;

	for (int j = 0; j <= n; j++) {
		for (int k = 0; j + k <= n; k++) {
			int i = n - j - k;
			int cpIndex = betri::cpIndex(j, k, n);

			subFormula = " + " + std::to_string(betri::FACTORIALS[n] /
				(betri::FACTORIALS[i] * betri::FACTORIALS[j] * betri::FACTORIALS[k]));
			subFormula += " * bt.cps[" + std::to_string(cpIndex) + "]";

			for (int tmp = 0; tmp < i; tmp++) {
				subFormula += " * s";
			}
			for (int tmp = 0; tmp < j; tmp++) {
				subFormula += " * t";
			}

			for (int tmp = 0; tmp < k; tmp++) {
				posFormulaVec.clear();
				negFormulaVec.clear();

				// https://stackoverflow.com/questions/53849/how-do-i-tokenize-a-string-in-c
				size_t start = plusDelimiter.size(), end = 0, endPos = 0, endNeg = 0;

				bool next = subFormula.find(minusDelimiter, 0) < start;
				while (endNeg != std::string::npos) {
					endNeg = subFormula.find(minusDelimiter, start);
					endPos = subFormula.find(plusDelimiter, start);
					end = std::min(endNeg, endPos);

					if (next) {
						negFormulaVec.push_back(subFormula.substr(start,
							(end == std::string::npos) ? std::string::npos : end - start)
						);
					} else {
						posFormulaVec.push_back(subFormula.substr(start,
							(end == std::string::npos) ? std::string::npos : end - start)
						);
					}
					start = ((end > (std::string::npos - plusDelimiter.size()))
						? std::string::npos : end + plusDelimiter.size()
						);
					next = endNeg < endPos;
				}

				/*
				for (auto s : negFormulaVec) {
					std::cerr << "Neg: " << tmp << " -> " << s << std::endl;
				}
				for (auto s : posFormulaVec) {
					std::cerr << "Pos: " << tmp << " -> " << s << std::endl;
				}
				*/

				tmpFormula.clear();
				for (auto s : negFormulaVec) {
					tmpFormula += " - 1.0 * " + s;
					tmpFormula += " + " + s;
					auto pos = tmpFormula.length() - (s.length() - s.find(" * t"));
					if (s.find(" * t") == std::string::npos)
						tmpFormula += " * s";
					else
						tmpFormula.insert(pos, " * s");
					tmpFormula += " + " + s + " * t";
				}
				for (auto s : posFormulaVec) {
					tmpFormula += " + 1.0 * " + s;
					tmpFormula += " - " + s;
					auto pos = tmpFormula.length() - (s.length() - s.find(" * t"));
					if (s.find(" * t") == std::string::npos)
						tmpFormula += " * s";
					else
						tmpFormula.insert(pos, " * s");
					tmpFormula += " - " + s + " * t";
				}

				subFormula = tmpFormula;
				/*
				if (cpIndex == 3) {
					std::cerr << "//////////////" << tmp << std::endl;
					std::cerr << subFormula << "\n" << std::endl;

					std::cerr << "Negative" << std::endl;
					for (auto elem : negFormulaVec)
						std::cerr << elem << std::endl;
					std::cerr << "Positive" << std::endl;
					for (auto elem : posFormulaVec)
						std::cerr << elem << std::endl;
					std::cerr << std::endl;
				}
				*/
			}

			//std::cerr << "form " << subFormula << std::endl;

			//std::cerr << " j " << j << " k " << k << " " << subFormula << std::endl;

			subFormula += "\n";
			formula += subFormula;
		}
	}

	//s.erase(std::find(s.begin(), s.end(), ' '));
	// https://en.cppreference.com/w/cpp/regex/regex_replace
	// http://www.cplusplus.com/reference/regex/ECMAScript/
	// http://www.informit.com/articles/article.aspx?p=2064649&seqNum=2

	// Replace additional 1.0
	std::regex replace_re0("1\\.0+\\s\\*\\s");
	formula = std::regex_replace(formula, replace_re0, "");

	// Replace additional zeros?
	std::regex replace_re1("\\.0+\\s");
	formula = std::regex_replace(formula, replace_re1, ".0 ");

	//std::cerr << "\nfinal formula" << std::endl;
	//std::cerr << formula << std::endl;

	std::regex replace_re2("\\n");
	formula = std::regex_replace(formula, replace_re2, "");


	std::vector<std::string> theStringVector;
	size_t start = 0, end = 0;
	while (end != std::string::npos) {
		end = std::min(
			formula.find(minusDelimiter, start + minusDelimiter.size()),
			formula.find(plusDelimiter, start + minusDelimiter.size())
		);

		// If at end, use length=maxLength.  Else use length=end-start.
		theStringVector.push_back(formula.substr(start,
			(end == std::string::npos) ? std::string::npos : end - start));

		// If at end, use start=maxSize.  Else use start=end+delimiter.
		start = ((end > (std::string::npos - minusDelimiter.size()))
			? std::string::npos : end + minusDelimiter.size());
		start = end;
	}

	std::vector<std::string> regexVec;
	std::string myRegex;
	for (int i = 0; i <= n; i++) {
		for (int j = 0; j <= n - i; j++) {
			myRegex.clear();
			for (int x = 0; x < i; x++) {
				myRegex += " * s";
			}
			for (int y = 0; y < j; y++) {
				myRegex += " * t";
			}
			regexVec.push_back(myRegex);
		}
	}

	// TODO reserve oben auch
	std::vector<std::string> qVec;

	std::string tmp;
	std::string save;
	for (int iter = regexVec.size() - 1; iter != 0; iter--) {
		tmp.clear();
		for (int stringIt = 0; stringIt != theStringVector.size(); stringIt++) {
			if (theStringVector[stringIt].find(regexVec[iter], 0) != std::string::npos) {
				theStringVector[stringIt].erase(
					theStringVector[stringIt].end() - regexVec[iter].size(),
					theStringVector[stringIt].end()
				);
				tmp += theStringVector[stringIt];
				// remove used ones
				theStringVector[stringIt] = "";
			}
		}
		qVec.push_back(tmp);
	}
	tmp.clear();
	for (auto s : theStringVector) {
		tmp += s;
	}
	qVec.push_back(tmp);

	std::string combinedQString;
	for (int stringIt = 0; stringIt != qVec.size(); stringIt++) {
		combinedQString += "vec3 q_" + std::to_string(stringIt) + " =" + qVec[stringIt] + ";\n";
	}

	std::string combinedBuv = "B_uv =";
	for (int stringIt = 0; stringIt != regexVec.size(); stringIt++) {
		combinedBuv += " + q_" + std::to_string(stringIt) + regexVec[regexVec.size() - stringIt - 1] + "\n";
	}
	combinedBuv += ";"; // TODO

	////////////////////////////////////
	// Calculate partial derivate (s) //
	////////////////////////////////////
	// TODO reserve
	std::vector<std::string> derivateVec;
	std::string derivate;
	for (int i = 0; i <= n; i++) {
		for (int j = 0; j <= n - i; j++) {
			if (i == 0) {
				derivateVec.push_back("-1");
				continue;
			}
			derivate.clear();
			if (i >= 2) {
				derivate += " * " + std::to_string(i);
			}
			for (int x = 0; x < i - 1; x++) {
				derivate += " * s";
			}
			for (int y = 0; y < j; y++) {
				derivate += " * t";
			}
			derivateVec.push_back(derivate);
		}
	}

	std::string dBs = "dBs =";
	for (int stringIt = 0; stringIt != derivateVec.size(); stringIt++) {
		if (derivateVec[derivateVec.size() - stringIt - 1] != "-1")
			dBs += " + q_" + std::to_string(stringIt) + derivateVec[derivateVec.size() - stringIt - 1] + "\n";
	}
	dBs += ";"; // TODO

	////////////////////////////////////
	// Calculate partial derivate (t) //
	////////////////////////////////////
	derivateVec.clear();
	for (int i = 0; i <= n; i++) {
		for (int j = 0; j <= n - i; j++) {
			if (j == 0) {
				derivateVec.push_back("-1");
				continue;
			}
			derivate.clear();
			if (j >= 2) {
				derivate += " * " + std::to_string(j);
			}
			for (int x = 0; x < i; x++) {
				derivate += " * s";
			}
			for (int y = 0; y < j - 1; y++) {
				derivate += " * t";
			}
			derivateVec.push_back(derivate);
		}
	}

	std::string dBt = "dBt =";
	for (int stringIt = 0; stringIt != derivateVec.size(); stringIt++) {
		if (derivateVec[derivateVec.size() - stringIt - 1] != "-1")
			dBt += " + q_" + std::to_string(stringIt) + derivateVec[derivateVec.size() - stringIt - 1] + "\n";
	}
	dBt += ";"; // TODO

	/*
	std::cerr << "\n\nQs" << std::endl;
	std::cerr << combinedQString << std::endl;

	std::cerr << "\ndBs" << std::endl;
	std::cerr << dBs << std::endl;

	std::cerr << "\ndBt" << std::endl;
	std::cerr << dBt << std::endl;

	std::cerr << "\nBuv" << std::endl;
	std::cerr << combinedBuv << std::endl;
	*/

	///////////////////////////
	// Write results to file //
	///////////////////////////
	// http://openflipper.org/Daily-Builds/Doc/Staging/Developer/a02657.html
	// https://stackoverflow.com/questions/9505085/replace-a-line-in-text-file
	std::ifstream filein(shaderDir + "/BezierTriangle/raytracing_template_fs.glsl");
	std::ofstream fileout(shaderDir + "/BezierTriangle/raytracing_gen_fs.glsl");

	if (!filein) {
		std::cerr << "Error open filein!" << std::endl;
		return;
	}
	if (!fileout) {
		std::cerr << "Error open fileout!" << std::endl;
		return;
	}

	std::string strTemp;
	//bool found = false;
	while (std::getline(filein, strTemp)) {
		fileout << strTemp << "\n";
		if (strTemp.find("BEGIN QS") != std::string::npos) {
			fileout << combinedQString;
		} else if (strTemp.find("BEGIN DBS") != std::string::npos) {
			fileout << dBs;
		} else if (strTemp.find("BEGIN DBT") != std::string::npos) {
			fileout << dBt;
		} else if (strTemp.find("BEGIN BUV") != std::string::npos) {
			fileout << combinedBuv;
		}
	}

	// TODO clear all vectors
}