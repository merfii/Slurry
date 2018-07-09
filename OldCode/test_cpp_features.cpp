#include <iostream>
#include <functional>
#include <thread>
#include <vector>


using namespace std;
using namespace placeholders;
#include <chrono>

namespace test{
	struct X
	{
		bool foo(int a) { cout << a << endl; return false; }
	};

	int test1()
	{
		X x;

		auto func = bind(&X::foo, &x, _1);
		func(5);
		return 0;
	}


	void test2()
	{

		chrono::high_resolution_clock::time_point tp1 = chrono::high_resolution_clock::now();
		chrono::milliseconds ms{ 3 }; // 3 毫秒
		// 6000 microseconds constructed from 3 milliseconds
		chrono::microseconds us = 2 * ms; //6000微秒
		// 30Hz clock using fractional ticks
		chrono::duration<double, ratio<1, 30>> hz30(3.5);
		cout << "3 ms duration has " << ms.count() << " ticks\n" << "6000 us duration has " << us.count() << " ticks\n";
		chrono::high_resolution_clock::time_point tp2 = chrono::high_resolution_clock::now();


		cout << chrono::duration_cast<chrono::microseconds>(tp2 - tp1).count() << endl;

		auto tnow = chrono::system_clock::to_time_t
			(chrono::system_clock::now());
		struct tm* ptm = localtime(&tnow);
		cout << ptm->tm_year << endl;


	}



	void bar(int x)
	{
		vector<int> num;
		num.resize(x);
		for (int j : num)
		{
			cout << j << endl;
			this_thread::sleep_for(chrono::seconds(1));
		}
		// do stuff...

	}
	void test3()
	{
		thread *hrd = new thread(bar, 12);  // spawn new thread that calls bar(0)
		hrd->detach();
		cout << "main, foo and bar now execute concurrently...\n";
		//hrd->join();               // pauses until second finishes


	}

}/*

#include <atomic>
atomic<int> uni_count;
atomic_flag uni_flag = ATOMIC_FLAG_INIT;
int loop_left = 10;
void func(int id)
{
	while (1)
	{	
		uni_count.fetch_add(1);
		cout << "#" << id << ": "<<uni_count << endl;

		this_thread::sleep_for(chrono::milliseconds(300));

		while (uni_flag.test_and_set());
		//Critical section
		loop_left--;
		if (loop_left < 0)
		{
			uni_flag.clear();
			break;
		}

		//Critical End
		uni_flag.clear();
	}

}

void test_atomic()
{
	thread a(func, 1);
	thread b(func, 2);
	a.join();
	b.join();
	
}

#include <string>
#include <algorithm>
struct pyUnit {     // 拼音单元数据结构
	string word;
	string py;
	int Pert;  // 单词 拼音 百分比
};
bool cmp(const pyUnit a, const pyUnit b)   //  比较函数cmp
{
	return a.Pert < b.Pert;
}
void test_lambda()
{
	vector<pyUnit> sList;
	sort(sList.begin(), sList.end(), cmp);     // 按Pert排序

	for (auto it = sList.begin(); it != sList.end(); ++it)  // 遍历输出
		cout << it->word << it->py << it->Pert << endl;

	cout << "=========================" << endl;

	//使用C++11 Lambda 函数，按拼音排序
	sort(sList.begin(), sList.end(),
		[](const pyUnit a, const pyUnit b) { return a.py < b.py; });

	//使用C++11 Lambda 函数，遍历输出
	for_each(sList.begin(), sList.end(),
		[](const pyUnit i) { cout << i.word << i.py << i.Pert << endl; });


}

#include<regex>

void test_regex()
{
	const char *reg_esp = "[^ ,.\\t\\n;:]+";  // 非分隔符

	regex rgx(reg_esp);  // regex 是模板类 basic_regex 以 char 特化的实体
	
	char *target = "Polytechnic University of Turin ";

	size_t n;
	do
	{
		cmatch match;  // cmatch 是模板类 match_results' 以 const char * 特化的实体
		regex_search(target, match, rgx);		//一次只匹配一个
		n = match.size();
		if (n > 0)
		{
			string str(match[0].first, match[0].second);
			cout << str << "\n";
			target = const_cast<char*>(match[0].second + 1);
		}
	} while (n > 0);

	cout << endl;

	std::string s = "Some people, when confronted with a problem, think "
		"\"I know, I'll use regular expressions.\" "
		"Now they have two problems.";

	std::regex self_regex("REGULAR EXPRESSIONS",
		std::regex_constants::ECMAScript | std::regex_constants::icase);
	if (std::regex_search(s, self_regex)) {
		std::cout << "Text contains the phrase 'regular expressions'\n";
	}

	std::regex word_regex("(\\S+)");
	auto words_begin =
		std::sregex_iterator(s.begin(), s.end(), word_regex);
	auto words_end = std::sregex_iterator();

	std::cout << "Found "
		<< std::distance(words_begin, words_end)
		<< " words\n";

	const int N = 6;
	std::cout << "Words longer than " << N << " characters:\n";
	for (std::sregex_iterator i = words_begin; i != words_end; ++i) {
		std::smatch match = *i;
		std::string match_str = match.str();
		if (match_str.size() > N) {
			std::cout << "  " << match_str << '\n';
		}
	}

	std::regex long_word_regex("(\\w{7,})");
	std::string new_s = std::regex_replace(s, long_word_regex, "[$&]");
	std::cout << new_s << '\n';
}
*/

/*

查找特定扩展名文件
// get a list of all html files in the current directory
QStringList filter;
filter << "*.htm";
filter << "*.html";

QStringList htmlFiles = QDir(":/").entryList(filter, QDir::Files);

QTextStream out(stdout);

if (htmlFiles.isEmpty()) {
out << "No html files available.";
return 1;
}

// parse each html file and write the result to file/stream
foreach(QString file, htmlFiles)
parseHtmlFile(out, ":/" + file);

*/

#include <opencv2\opencv.hpp>
void test_opencv()
{
	static int count;
	char win_title[10];
	cv::Mat mat;
	mat.create(400, 400, CV_8UC3);
	cv::circle(mat, cv::Point(200, 200),50, cv::Scalar(0, 200, 20), 3);
	sprintf_s(win_title, "Window %d", count++);
	cv::imshow(win_title, mat);
	cv::waitKey(-1);

}

/*

命令行文件读取与输出
QTextStream errorStream(stderr);

if (argc != 2)
{
errorStream << XmlStreamLint::tr(
"Usage: xmlstreamlint <path to XML file>\n");
return ArgumentError;
}

QString inputFilePath(QCoreApplication::arguments().at(1));
QFile inputFile(inputFilePath);

if (!QFile::exists(inputFilePath))
{
errorStream << XmlStreamLint::tr(
"File %1 does not exist.\n").arg(inputFilePath);
return FileFailure;

} else if (!inputFile.open(QIODevice::ReadOnly)) {
errorStream << XmlStreamLint::tr(
"Failed to open file %1.\n").arg(inputFilePath);
return FileFailure;
}

QFile outputFile;
if (!outputFile.open(stdout, QIODevice::WriteOnly))
{
errorStream << XmlStreamLint::tr("Failed to open stdout.");
return WriteError;
}


*


void stateMachineInit()
{
QStateMachine *machine = new QStateMachine;
QState *s1 = new QState();
QState *s2 = new QState();
QState *s3 = new QState();
s1->assignProperty(ui.logText, "geometry", QRectF(100, 100, 100, 60));
s2->assignProperty(ui.logText, "geometry", QRectF(300, 200, 200, 200));
s3->assignProperty(ui.logText, "geometry", QRectF(50, 250, 150, 400));

QSignalTransition *tran = s1->addTransition(ui.testButton1, SIGNAL(clicked()), s2);
QPropertyAnimation *animation = new QPropertyAnimation(ui.logText, "geometry");
animation->setEasingCurve(QEasingCurve::OutCubic);
animation->setDuration(1000);
tran->addAnimation(animation);

tran = s2->addTransition(ui.testButton1, SIGNAL(clicked()), s3);
animation = new QPropertyAnimation(ui.logText, "geometry");
animation->setEasingCurve(QEasingCurve::OutBack);
animation->setDuration(800);
tran->addAnimation(animation);

tran = s3->addTransition(ui.testButton1, SIGNAL(clicked()), s1);
animation = new QPropertyAnimation(ui.logText, "geometry");
animation->setEasingCurve(QEasingCurve::OutCubic);
animation->setDuration(5500);
//tran->addAnimation(animation);

animation->setEasingCurve(QEasingCurve::OutCubic);
animation->setDuration(500);
machine->addDefaultAnimation(animation);
machine->addState(s1);
machine->addState(s2);
machine->addState(s3);
machine->setInitialState(s1);
machine->start();


QParallelAnimationGroup *group = new QParallelAnimationGroup;
for (int i = 0; i < items.count(); ++i) {
QPropertyAnimation *anim = new QPropertyAnimation(items[i], "pos");
anim->setDuration(750 + i * 25);
anim->setEasingCurve(QEasingCurve::InOutBack);
group->addAnimation(anim);
}

}
*/