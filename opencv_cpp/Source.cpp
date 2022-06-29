#include "MainForm.h"

using namespace System;
using namespace System::Windows::Forms;
using namespace opencvcpp;

[STAThreadAttribute]
int main() {
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	Application::Run(gcnew MainForm());
	return 0;
}