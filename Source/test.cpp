#include <cstdlib>
#include <iostream>
#include "parameters.h"
#include "autoscript.h"

using namespace std;

void TestParameters() {
    char string_p[100] = {0};
    int int_p = 0;
    float float_p = 0.0;
    double double_p = 0.0;
    char string_p2[100] = "test string";
    int int_p2 = 55;
    float float_p2 = 6.6;
    double double_p2 = 7.7;

    cout << "\n";

	Parameters *parameters_ = new Parameters("param1.par");
	if (!parameters_->file_opened_) {
        cout << "Error opening param1 file!\n";
    }
	if (!parameters_->ReadValues()) {
        cout << "Error reading param1 file!\n";
    }
	parameters_->Close();
	if (!parameters_->GetValue("StringP", string_p)) {
        cout << "Error reading StringP!\n";
    }
	if (!parameters_->GetValue("IntP", &int_p)) {
        cout << "Error reading IntP!\n";
    }
    delete parameters_;
    cout << "StringP: " << string_p << "\n";
    cout << "IntP: " << int_p << "\n";
		
	parameters_ = new Parameters("param2.par");
	if (!parameters_->file_opened_) {
        cout << "Error opening param2 file!\n";
    }
	if (!parameters_->ReadValues()) {
        cout << "Error reading param2 file!\n";
    }
	parameters_->Close();
	if (!parameters_->GetValue("FloatP", &float_p)) {
        cout << "Error reading FloatP!\n";
    }
	
	if (!parameters_->GetValue("DoubleP", &double_p)) {
        cout << "Error reading DoubleP!\n";
    } 
	if (!parameters_->GetValue("XXX", string_p2)) {
        cout << "Error reading XXX!\n";
    }
	if (!parameters_->GetValue("XXX", &int_p2)) {
        cout << "Error reading XXX!\n";
    }
	if (!parameters_->GetValue("XXX", &float_p2)) {
        cout << "Error reading XXX!\n";
    }
	if (!parameters_->GetValue("XXX", &double_p2)) {
        cout << "Error reading XXX!\n";
    }
    delete parameters_;
    cout << "FloatP: " << float_p << "\n";
    cout << "DoubleP: " << double_p << "\n";
    cout << "\n";
    cout << "StringP2: " << string_p2 << "\n";
    cout << "IntP2: " << int_p2 << "\n";
    cout << "FloatP2: " << float_p2 << "\n";
    cout << "DoubleP2: " << double_p2 << "\n";
}

void TestAutoScript() {
	
	AutoScript *autoscript_ = new AutoScript("auto1.as");
	if (!autoscript_->file_opened_) {
        cout << "Error opening auto1 file!\n";
    }
	if (!autoscript_->ReadScript()) {
        cout << "Error reading auto1 file!\n";
    }
	autoscript_->Close();
    delete autoscript_;
	
	autoscript_command ac = autoscript_->GetNextCommand();
	//autoscript_command ac = autoscript_->GetCommand(0);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";
	
	ac = autoscript_->GetNextCommand();
	//ac = autoscript_->GetCommand(1);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";

	ac = autoscript_->GetNextCommand();
	//ac = autoscript_->GetCommand(2);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";

	ac = autoscript_->GetNextCommand();
	//ac = autoscript_->GetCommand(3);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";

	ac = autoscript_->GetNextCommand();
	//ac = autoscript_->GetCommand(4);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";

	ac = autoscript_->GetNextCommand();
	//ac = autoscript_->GetCommand(5);
    cout << ac.command << "\n";
	cout << ac.param1 << "\n";
	cout << ac.param2 << "\n";
	cout << ac.param3 << "\n";
	cout << ac.param4 << "\n";
	cout << ac.param5 << "\n\n";

}

int main(int argc, char *argv[])
{
    //TestParameters();
	//TestAutoScript();
    system("PAUSE");
    return EXIT_SUCCESS;
}
