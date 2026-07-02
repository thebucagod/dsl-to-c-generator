#include <iostream>
#include "pugixml.hpp"
#include <string>
#include <stdexcept>
#include <vector>
#include <set>
#include <fstream>

using namespace pugi;

/*
<Block>:
	SID -- "адрес" при разборе связей (17#out:1 означает "выход блока с SID=17").
	Name — имя блока.
	BlockType — тип блока.

<P>:
	<P Name="Gain"> — только у блоков Gain.
		Становится константой умножения:
		nwocg.P_gain = nwocg.Add1 * 3; — вот это 3 берётся отсюда.
	<P Name="Inputs"> — только у блоков Sum.
		Строка вида "+-" — каждый символ это знак для соответствующего входа.
		Если параметра нет — все входы положительные.
<Line> (и <Branch>):
	Src — откуда идёт сигнал, формат SID#out:N.
		Нужен чтобы знать: "входящий сигнал этого блока — это выход блока с таким-то SID".
	Dst — куда приходит сигнал, формат SID#in:N.
		Нужен чтобы знать: "вот в какой вход какого блока это приходит".
	
	Из Src и Dst мы строим главную таблицу:
		для каждого блока — список его входов
		(какой блок подаёт данные на вход №1, какой на вход №2).
		Именно эта таблица используется при кодогенерации —
		чтобы написать nwocg.Add1 = nwocg.setpoint - nwocg.feedback,
		нужно знать что на вход №1 блока Add1 приходит сигнал от блока setpoint,
		а на вход №2 — от feedback.
*/

// teg: Block
struct Block
{
		std::string BlockType;
		std::string Name;
		std::string SID;
		std::string param;      // "+-"/"" для Sum, double для Gain, пусто для остальных
};

// teg: Line and Branch in one struct
struct Connection
{
	std::string srcSid;   // от кого
	std::string srcPort;  // с какого выхода
	std::string dstSid;   // кому
	std::string dstPort;  // на какой вход
};

// Вспомогательная структура для коннектора, которая
// хранить sid и port таких узлов как "sid#out:port", "sid#in:port"
struct sidPort
{
	std::string sid;
	std::string port;
};

struct parsedSystem {
	std::vector<Block> blocks;
	std::vector<Connection> connections;
};

// Пространство имен содержащее parse-функции
namespace parser {
	
	Block parseBlock(xml_node node);						// Возвращает структуру Block из соответсвующего узла в xml
	std::vector<Connection> parseConnection(xml_node node);	// возвращает вектор структур connection (обработанный List + branch) из соответсвующего узла в xml
	sidPort parseSidPort(std::string sp);					// Возвращает структуру sidPort из строки форматов "sid#out:port", "sid#in:port"
	parsedSystem parseSystem(xml_node systemNode);			// Возвращает parsedSystem из соответсвующего узла в xml

	std::string findSrc(xml_node node);						// Находим строку формата sid#out:port в атрибуте Name="Src"
	std::string findDst(xml_node node);						// Находим строку формата sid#out:port в атрибуте Name="Dst" 

	parsedSystem& topoSort(parsedSystem &ps);				// Функция сортирует std::vector<Block> blocks в parsedSystem
}

namespace generate {
	void generateCode(const parsedSystem& ps, const std::string& filename = "nwocg_run");	// Генерируем C-код
	std::string toValidName(const std::string& name);										// Убирает пробелы из "Unit Delay1" подобных строк
	std::string generateRhs(const parsedSystem& ps, const Block &block);					// Правая часть "=" в void nwocg_generated_step()
}

/* ====================== PARSE-FUNCTIONS ====================== */
Block parser::parseBlock(xml_node node) {
	if (std::string(node.name()) != "Block")	{
		throw std::invalid_argument("Expected 'Block' node, got: " + std::string(node.name()));
	}

	Block block;
	block.BlockType = node.attribute("BlockType").value();
	block.Name = node.attribute("Name").value();
	block.SID = node.attribute("SID").value();
	
	if (block.BlockType == "Sum") {
		for (xml_node iter : node.children("P")) {
			if (std::string(iter.attribute("Name").value()) == "Inputs") {
				block.param = iter.child_value();
				return block;
			}
		}
		return block;
	}

	if (block.BlockType == "Gain")
	{
		for (xml_node iter : node.children("P")) {
			if (std::string(iter.attribute("Name").value()) == "Gain") {
				block.param = iter.child_value();
				return block;
			}
		}
		return block;
	}

	return block;
}

std::vector<Connection> parser::parseConnection(xml_node node) {
	if (std::string(node.name()) != "Line") {
		throw std::invalid_argument("Expected 'Line' node, got: " + std::string(node.name()));
	}

	sidPort srcSidPort = parseSidPort(findSrc(node));

	std::vector<sidPort> dstSidPort;
	bool hasBranches = false;

	for (xml_node iter : node.children("Branch")) {
		hasBranches = true;
		dstSidPort.push_back(parseSidPort(findDst(iter)));
	}

	if (!hasBranches) {
		dstSidPort.push_back(parseSidPort(findDst(node)));
	}

	std::vector<Connection> connectionVector;

	for (auto iter : dstSidPort) {
		Connection connection;
		connection.srcSid = srcSidPort.sid;
		connection.srcPort = srcSidPort.port;

		connection.dstSid = iter.sid;
		connection.dstPort = iter.port;

		connectionVector.push_back(connection);
	}

	return connectionVector;
}

sidPort parser::parseSidPort(std::string sp) {
	size_t hashPos = sp.find('#');
	size_t colonPos = sp.find(':');

	sidPort sidPort;

	sidPort.sid = sp.substr(0, hashPos);
	sidPort.port = sp.substr(colonPos + 1);

	return sidPort;
}

std::string parser::findSrc(xml_node node) {
	if (std::string(node.name()) != "Line") {
		throw std::invalid_argument("Expected 'Line' node, got: " + std::string(node.name()));
	}

	for (xml_node iter : node.children("P")) {
		if (std::string(iter.attribute("Name").value()) == "Src") {
			return std::string(iter.child_value());
		}
	}
	return "";
}

std::string parser::findDst(xml_node node) {
	if (std::string(node.name()) != "Line" && std::string(node.name()) != "Branch") {
		throw std::invalid_argument("Expected 'Line' or 'Branch' node, got: " + std::string(node.name()));
	}

	for (xml_node iter : node.children("P")) {
		if (std::string(iter.attribute("Name").value()) == "Dst") {
			return std::string(iter.child_value());

		}
	}
	return "";
}

parsedSystem parser::parseSystem(xml_node systemNode) {
	if (std::string(systemNode.name()) != "System") {
		throw std::invalid_argument("Expected 'System' node, got: " + std::string(systemNode.name()));
	}

	parsedSystem result;

	// Находим все узлы с тегом Block и добавляем их в наш вектор блоков
	for (xml_node block : systemNode.children("Block")) {
		result.blocks.push_back(parseBlock(block));
	}
	for (xml_node connection : systemNode.children("Line")) {
		std::vector<Connection> connections = parseConnection(connection);
		for (auto& c : connections) {
			result.connections.push_back(c);
		}
	}

	return result;
}

/* ====================== SORT-FUNCTION ====================== */
parsedSystem& parser::topoSort(parsedSystem& ps) {
	std::vector<Block> sorted;
	std::set<std::string> doneSids;

	for (Block& b : ps.blocks) {
		if (b.BlockType == "Inport" || b.BlockType == "UnitDelay") {
			sorted.push_back(b);
			doneSids.insert(b.SID);
		}
	}

	while (sorted.size() < ps.blocks.size()) {
		for (Block& b : ps.blocks) {
			if (doneSids.count(b.SID) > 0) continue;

			bool allInputsReady = true;
			for (Connection& c : ps.connections) {
				if (c.dstSid == b.SID) {
					if (doneSids.count(c.srcSid) == 0) {
						allInputsReady = false;
						break;
					}
				}
			}

			if (allInputsReady) {
				sorted.push_back(b);
				doneSids.insert(b.SID);
			}
		}
	}

	ps.blocks = sorted;
	return ps;
}

/* ====================== GENERATE-FUNCTIONS ====================== */
std::string findSrcName(const parsedSystem& ps, const std::string& blockSid, const std::string& portNum = "1");

void generate::generateCode(const parsedSystem& ps, const std::string& filename) {
	std::string baseName = filename;

	std::ofstream osc(baseName + ".c");
	std::ofstream osh(baseName + ".h");

	if (!osc || !osh) {
		std::cerr << "Ошибка создания файлов!" << std::endl;
		return;
	}

	// ========== HEADER (.h) ==========
	osh << "#ifndef NWOCG_RUN_H" << std::endl;
	osh << "#define NWOCG_RUN_H" << std::endl;

	osh << "\n";

	osh << "#include <stddef.h>" << std::endl;

	osh << "\n";

	osh << "struct nwocg_ExtPort" << std::endl;
	osh << "{" << std::endl;
	osh << "\tconst char* name;" << std::endl;
	osh << "\tdouble* address;" << std::endl;
	osh << "\tint dir;" << std::endl;
	osh << "};" << std::endl;

	osh << "\n";

	osh << "#endif";

	// ========== SOURCE (.c) ==========
	// Подключаем заголовный файл
	osc << "#include \"" << baseName << ".h\"\n";
	osc << "#include <math.h>\n\n";

	osc << "\n";

	// Структура nwocg со всеми блоками кроме последнего
	osc << "static struct" << std::endl;
	osc << "{" << std::endl;
	for (const Block& block : ps.blocks) {
		if (block.BlockType != "Outport") {
			osc << "\tdouble " << toValidName(block.Name) << ";" << std::endl;
		}
	}
	osc << "} nwocg;" << std::endl;

	osc << "\n";

	// void nwocg_generated_init() -- инициализируем все UnitDelay
	osc << "void nwocg_generated_init()" << std::endl;
	osc << "{" << std::endl;
	for (const Block& block : ps.blocks) {
		if (block.BlockType == "UnitDelay") {
			osc << "\tnwocg." + toValidName(block.Name) << " = 0;" << std::endl;
		}
	}
	osc << "}" << std::endl;

	osc << "\n";


	// void nwocg_generated_step() -- функция "шага" с присваиванием значений каждой переменной кроме BlockType="Inport"
	osc << "void nwocg_generated_step()" << std::endl;
	osc << "{" << std::endl;
	for (const Block& block : ps.blocks) {
		if (block.BlockType != "Inport" && block.BlockType != "Outport" && block.BlockType != "UnitDelay") {
			osc << "\tnwocg." << block.Name << " = " << generateRhs(ps, block) << std::endl;
		}
	}

	osc << "\n";

	for (const Block& block : ps.blocks) {
		if (block.BlockType == "UnitDelay") {
			osc << "\tnwocg." << toValidName(block.Name) << " = nwocg." << findSrcName(ps, block.SID) << ";" << std::endl;
		}
	}
	osc << "}" << std::endl;

	osc << "\n";

	// ext_ports
	osc << "static const nwocg_ExtPort" << std::endl;
	osc << "\text_ports[] =" << std::endl;
	osc << "{" << std::endl;

	for (const Block& block : ps.blocks) {
		if (block.BlockType == "Outport") {
			std::string portType = (block.BlockType == "Outport") ? "0" : "1";
			std::string blockName = block.Name;

			osc << "\t{ \"" << block.Name << "\", &nwocg." << findSrcName(ps, block.SID)
				<< ", " << portType << " }," << std::endl;
		}
	}

	for (const Block& block : ps.blocks) {
		if (block.BlockType == "Inport") {
			std::string portType = (block.BlockType == "Outport") ? "0" : "1";
			std::string blockName = block.Name;

			osc << "\t{ \"" << block.Name << "\", &nwocg." << blockName
				<< ", " << portType << " }," << std::endl;
		}
	}
	osc << "\t{ 0, 0, 0 }," << std::endl;
	
	osc << "};" << std::endl;

	osc << "\n";

	// Ссылки (указатель на матрицу и размер матрицы)
	osc << "const nwocg_ExtPort * const\n";
	osc << "    nwocg_generated_ext_ports = ext_ports;\n\n";

	osc << "const size_t\n";
	osc << "    nwocg_generated_ext_ports_size = sizeof(ext_ports);\n";
}

std::string generate::toValidName(const std::string& name) {
	std::string result = name;
	result.erase(std::remove(result.begin(), result.end(), ' '), result.end());
	return result;
}

std::string generate::generateRhs(const parsedSystem& ps, const Block& block) {
	if (block.BlockType == "Sum") {
		std::string srcName1Port = findSrcName(ps, block.SID, "1");
		std::string srcName2Port = findSrcName(ps, block.SID, "2");

		char sign1 = block.param.empty() ? '+' : block.param[0];
		char sign2 = block.param.size() > 1 ? block.param[1] : '+';

		std::string rhs = "nwocg." + toValidName(srcName1Port) + " " + std::string(1, sign2) + " nwocg." + toValidName(srcName2Port) + ";";
		if (sign1 == '-') rhs = std::string(1, sign1) + rhs;
		return rhs;
	}

	if (block.BlockType == "Gain") {
		return "nwocg." + findSrcName(ps, block.SID) + " * " + block.param + ";";
	}

	return "";
}

/* ИЩЕМ NAME ПРОШЛОГО БЛОКА ДЛЯ C-ФУНКЦИИ void nwocg_generated_step() */
std::string findSrcName(const parsedSystem& ps, const std::string& blockSid, const std::string& portNum) {
	for (const Connection& connection : ps.connections) {
		if (connection.dstSid == blockSid && connection.dstPort == portNum) {
			std::string srcSid = connection.srcSid;
			for (const Block& block : ps.blocks) {
				if (block.SID == srcSid) {
					return block.Name;
				}
			}
		}
	}
	throw std::invalid_argument("SID " + blockSid + " not found");
}

// Функция включащая все функции parser и возвращает готовый отсортированный объект parsedSystem
parsedSystem loadFile(std::string file) {
	xml_document doc;
	xml_parse_result result = doc.load_file(file.c_str());

	if (!result) {
		throw std::runtime_error("Failed to load file: " + std::string(result.description()));
	}

	xml_node system = doc.child("System");
	parsedSystem ps = parser::parseSystem(system);
	parser::topoSort(ps);
	return ps;
}



int main() {
	parsedSystem ps = loadFile("PI_Controller.xml");
	generate::generateCode(ps);

	return 0;
}
