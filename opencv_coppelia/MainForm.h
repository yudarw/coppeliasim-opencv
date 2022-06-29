#pragma once

namespace opencvcoppelia {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for MainForm
	/// </summary>
	public ref class MainForm : public System::Windows::Forms::Form
	{
	public:
		MainForm(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::Button^ button2;
	private: System::Windows::Forms::PictureBox^ pictureBox1;
	private: System::Windows::Forms::Button^ button3;
	private: System::Windows::Forms::Timer^ timer1;



	private: System::Windows::Forms::Button^ button6;
	private: System::Windows::Forms::Button^ button7;
	private: System::Windows::Forms::Button^ button8;
	private: System::Windows::Forms::GroupBox^ groupBox1;
	private: System::Windows::Forms::Button^ button10;
	private: System::Windows::Forms::Button^ button9;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::TextBox^ tb_robot_name;
	private: System::Windows::Forms::GroupBox^ groupBox2;
	private: System::ComponentModel::IContainer^ components;
	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->button6 = (gcnew System::Windows::Forms::Button());
			this->button7 = (gcnew System::Windows::Forms::Button());
			this->button8 = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->button10 = (gcnew System::Windows::Forms::Button());
			this->button9 = (gcnew System::Windows::Forms::Button());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->tb_robot_name = (gcnew System::Windows::Forms::TextBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->BeginInit();
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(462, 319);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(196, 39);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Sim Connect";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MainForm::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(19, 77);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(159, 23);
			this->button2->TabIndex = 1;
			this->button2->Text = L"Read Vision Sensor";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MainForm::button2_Click);
			// 
			// pictureBox1
			// 
			this->pictureBox1->BorderStyle = System::Windows::Forms::BorderStyle::Fixed3D;
			this->pictureBox1->Location = System::Drawing::Point(12, 12);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(435, 346);
			this->pictureBox1->TabIndex = 2;
			this->pictureBox1->TabStop = false;
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(19, 19);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(159, 23);
			this->button3->TabIndex = 3;
			this->button3->Text = L"Read Proximity Sensor";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &MainForm::button3_Click);
			// 
			// timer1
			// 
			this->timer1->Tick += gcnew System::EventHandler(this, &MainForm::timer1_Tick);
			// 
			// button6
			// 
			this->button6->Location = System::Drawing::Point(19, 78);
			this->button6->Name = L"button6";
			this->button6->Size = System::Drawing::Size(159, 23);
			this->button6->TabIndex = 7;
			this->button6->Text = L"Read Robot Position";
			this->button6->UseVisualStyleBackColor = true;
			this->button6->Click += gcnew System::EventHandler(this, &MainForm::button6_Click);
			// 
			// button7
			// 
			this->button7->Location = System::Drawing::Point(19, 107);
			this->button7->Name = L"button7";
			this->button7->Size = System::Drawing::Size(159, 23);
			this->button7->TabIndex = 8;
			this->button7->Text = L"Read Robot Joint Position";
			this->button7->UseVisualStyleBackColor = true;
			this->button7->Click += gcnew System::EventHandler(this, &MainForm::button7_Click);
			// 
			// button8
			// 
			this->button8->Location = System::Drawing::Point(19, 48);
			this->button8->Name = L"button8";
			this->button8->Size = System::Drawing::Size(159, 23);
			this->button8->TabIndex = 9;
			this->button8->Text = L"Read Force Sensor";
			this->button8->UseVisualStyleBackColor = true;
			this->button8->Click += gcnew System::EventHandler(this, &MainForm::button8_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->button10);
			this->groupBox1->Controls->Add(this->button9);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->tb_robot_name);
			this->groupBox1->Controls->Add(this->button6);
			this->groupBox1->Controls->Add(this->button7);
			this->groupBox1->Location = System::Drawing::Point(462, 12);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(196, 178);
			this->groupBox1->TabIndex = 10;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Arm Robot ";
			// 
			// button10
			// 
			this->button10->Location = System::Drawing::Point(19, 136);
			this->button10->Name = L"button10";
			this->button10->Size = System::Drawing::Size(159, 23);
			this->button10->TabIndex = 13;
			this->button10->Text = L"Move Robot Position";
			this->button10->UseVisualStyleBackColor = true;
			this->button10->Click += gcnew System::EventHandler(this, &MainForm::button10_Click);
			// 
			// button9
			// 
			this->button9->Location = System::Drawing::Point(19, 49);
			this->button9->Name = L"button9";
			this->button9->Size = System::Drawing::Size(159, 23);
			this->button9->TabIndex = 12;
			this->button9->Text = L"Initialization";
			this->button9->UseVisualStyleBackColor = true;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(16, 26);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(70, 13);
			this->label1->TabIndex = 11;
			this->label1->Text = L"Model Name:";
			// 
			// tb_robot_name
			// 
			this->tb_robot_name->Location = System::Drawing::Point(92, 23);
			this->tb_robot_name->Name = L"tb_robot_name";
			this->tb_robot_name->Size = System::Drawing::Size(86, 20);
			this->tb_robot_name->TabIndex = 9;
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->button3);
			this->groupBox2->Controls->Add(this->button8);
			this->groupBox2->Controls->Add(this->button2);
			this->groupBox2->Location = System::Drawing::Point(462, 196);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(196, 117);
			this->groupBox2->TabIndex = 11;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Sensor";
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(671, 368);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->pictureBox1);
			this->Controls->Add(this->button1);
			this->MaximumSize = System::Drawing::Size(687, 407);
			this->MinimumSize = System::Drawing::Size(687, 407);
			this->Name = L"MainForm";
			this->Text = L"Coppeliasim Lib Test";
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pictureBox1))->EndInit();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->ResumeLayout(false);

		}
#pragma endregion

	public:
		void on_timer();
		void on_init();
		void btn_read_robot_joint_pos();
		void btn_read_force_sensor();
		void btn_sim_connect();
		void btn_sim_read_camera();
		void btn_read_proximity();
		void btn_remove_object();
		void btn_add_object();
		void btn_read_robot_pos();
		void btn_set_robot_pos();

	private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_sim_connect();
	}
	private: System::Void button2_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_sim_read_camera();
	}
	private: System::Void timer1_Tick(System::Object^ sender, System::EventArgs^ e) {
		on_timer();
	}
	private: System::Void MainForm_Load(System::Object^ sender, System::EventArgs^ e) {
		on_init();
	}
	private: System::Void button3_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_read_proximity();
	}
	private: System::Void button5_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_remove_object();
	}
	private: System::Void button4_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_add_object();
	}
	private: System::Void button6_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_read_robot_pos();
	}
	private: System::Void button7_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_read_robot_joint_pos();
	}
	private: System::Void button8_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_read_force_sensor();
	}
	private: System::Void button10_Click(System::Object^ sender, System::EventArgs^ e) {
		btn_set_robot_pos();
	}
};
}
