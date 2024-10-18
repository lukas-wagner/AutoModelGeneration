package modelGeneration;

import java.util.ArrayList;
import java.util.List;

import systemParameterExtraction.SaveDataModel;
import systemParameterExtraction.SystemParameterExtraction;
import systemParameterExtraction.TimeSeriesData;
import systemParameterModel.SystemParameters;

public class ModelGeneration {

	
	public static void main(String[] args) {
		new SaveDataModel(deriveSystemParameters());
		
		
	}
	
	public static SystemParameters deriveSystemParameters () {
		SystemParameters systemParameters = new SystemParameters(); 
		
		SystemParameterExtraction.getFilePathResourceDataList().addAll(setFilePaths());
		
		List<TimeSeriesData> timeSeriesData_UN_PreProcessed = SystemParameterExtraction.importData();
		
		
		systemParameters = SystemParameterExtraction.getSystemParameters();
		
		return systemParameters;
	}

	public static List<TimeSeriesData> setFilePaths () {

		List<TimeSeriesData> listOfFilePaths = new ArrayList();

		TimeSeriesData resource1 = new TimeSeriesData();
		resource1.setNameOfResource("gasfired_generator");
		resource1.setFilePath_TimeStamps("src/timeSeriesDataSet/resource1_timeStamps.csv");
		resource1.setFilePath_InputValues("src/timeSeriesDataSet/resource1_inputValues.csv");
		resource1.setFilePath_OutputValues("src/timeSeriesDataSet/resource1_outputValues.csv");
		resource1.setFilePath_SystemStates("src/timeSeriesDataSet/resource1_systemstates.csv");
		SystemParameterExtraction.getFilePathResourceDataList().add(resource1);


		TimeSeriesData resource2 = new TimeSeriesData();
		resource2.setNameOfResource("heat_exchanger");
		resource2.setFilePath_TimeStamps("src/timeSeriesDataSet/resource1_timeStamps.csv");
		resource2.setFilePath_InputValues("src/timeSeriesDataSet/resource1_outputValues.csv");
		resource2.setFilePath_OutputValues("src/timeSeriesDataSet/resource2_outputValues.csv");
		SystemParameterExtraction.getFilePathResourceDataList().add(resource2);

		SystemParameterExtraction.setFilePathFormProDesc("src/timeSeriesDataSet/chp_v1.json"); 
		return listOfFilePaths; 

	}
}
