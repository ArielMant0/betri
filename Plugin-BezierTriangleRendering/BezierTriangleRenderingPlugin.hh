#pragma once

#include <OpenFlipper/BasePlugin/BaseInterface.hh>
#include <OpenFlipper/BasePlugin/ToolboxInterface.hh>
#include <OpenFlipper/BasePlugin/LoggingInterface.hh>
#include <OpenFlipper/BasePlugin/RenderInterface.hh>
#include <OpenFlipper/BasePlugin/LoadSaveInterface.hh>

#include <OpenFlipper/common/Types.hh>

#include <qlineedit.h>
#include <qcheckbox.h>

class BezierTriangleRenderingPlugin : public QObject,
	BaseInterface,
	ToolboxInterface,
	LoggingInterface,
	LoadSaveInterface
{
	Q_OBJECT
	Q_INTERFACES(BaseInterface)
	Q_INTERFACES(ToolboxInterface)
	Q_INTERFACES(LoggingInterface)
	Q_INTERFACES(LoadSaveInterface)

	Q_PLUGIN_METADATA(IID "org.OpenFlipper.Plugins.Plugin-BezierTriangleRendering")

public:

	BezierTriangleRenderingPlugin() : m_tool(0) {}

	~BezierTriangleRenderingPlugin() {};

	QString name() { return QString("BezierTriangleRenderingPlugin"); };

	QString description()
	{
		return QString("Manage rendering settings for Bezier Triangles");
	};

private:

	QWidget *m_tool;

	////////////////////////////
	// Bench-File information //
	////////////////////////////
	QComboBox *filePathComboBox;
	QLineEdit *filePathLineEdit;
	QLineEdit *outFileLineEdit;
	QComboBox *fileTypeComboBox;

	QComboBox *testTypeComboBox;
	QComboBox *againstTypeComboBox;
	std::vector<QCheckBox*> perfCheckboxes;

///////////////////////////////////////////////////////////////////////////////
// Signals
///////////////////////////////////////////////////////////////////////////////
signals:

	void updateView();
    void updatedObject(int _identifier, const UpdateType &type);

	void log(Logtype _type, QString _message);
    void log(QString _message);

	void addEmptyObject(DataType _type, int& _id);

	void addToolbox(QString name, QWidget *widget, QIcon *icon);

///////////////////////////////////////////////////////////////////////////////
// Slots
///////////////////////////////////////////////////////////////////////////////
public slots:

	QString version() { return QString("1.0.0");
	};

private slots:

	void initializePlugin() override;

	// option setting function
	void setTessType(int);
	void setTessAmount(int);
	void setTessMode(int);
	void setBoundVType(int);
	void setVisulisationType(int);

	// TODO wirklich nötig?
	void setBError(int);
	void setDError(int);
	void setNewtonIt(int);

	void setCulling(bool);
	void setBoundVShow(bool);

};
