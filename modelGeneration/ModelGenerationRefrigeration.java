package modelGeneration;

import java.util.ArrayList;
import java.util.List;

import systemParameterExtraction.SaveDataModel;
import systemParameterExtraction.SystemParameterExtraction;
import systemParameterExtraction.TimeSeriesData;
import systemParameterModel.SystemParameters;

public class ModelGenerationRefrigeration {

	static final String ELECTRICITY = "Electricity";
	static final String COLDWATER = "ColdWater";

	public static void main(String[] args) {
		SystemParameters sysPara = deriveSystemParameters();
		new SaveDataModel(sysPara);

	}

	public static SystemParameters deriveSystemParameters () {
		SystemParameters systemParameters = new SystemParameters(); 
		//	
		SystemParameterExtraction.getFilePathResourceDataList().addAll(setFilePaths());
		//		List<Dependency> dep = SystemParameterExtraction.getDependencies();	

		List<TimeSeriesData> timeSeriesData_UN_PreProcessed = SystemParameterExtraction.importData();
		List<TimeSeriesData> timeSeriesData_PreProcessed =  SystemParameterExtraction.doPreProcessing(timeSeriesData_UN_PreProcessed);

		SystemParameterExtraction.setSystemParametersFromOtherMethods(timeSeriesData_PreProcessed);
		systemParameters = SystemParameterExtraction.getSystemParameters();

		return systemParameters;
	}

	public static List<TimeSeriesData> setFilePaths () {

		SystemParameterExtraction.setFilePathFormProDesc("src/input_refrigeration/fpb_refrigeration.json"); 

		SystemParameterExtraction.getTsdSystem().setFilePath_TimeStamps("src/input_refrigeration/timestamps.csv");
		SystemParameterExtraction.getTsdSystem().setFilePath_InputValues("src/input_refrigeration/system_input.csv");
//		SystemParameterExtraction.getTsdSystem().setEnergyCarriersInput(null);
		SystemParameterExtraction.getTsdSystem().setFilePath_OutputValues("src/input_refrigeration/system_output.csv");
		List<TimeSeriesData> listOfFilePaths = new ArrayList<TimeSeriesData>();

		TimeSeriesData compressor1 = new TimeSeriesData();
		compressor1.setNameOfResource("RefrigerationMachine1");
		compressor1.setFilePath_TimeStamps("src/input_refrigeration/timestamps.csv");

		List<String> energyCarrierInputComp1 = new ArrayList<String>(); 
		energyCarrierInputComp1.add(ELECTRICITY);
		compressor1.setEnergyCarriersInput(energyCarrierInputComp1);
		compressor1.setFilePath_InputValues("src/input_refrigeration/inp_comp1.csv");

		compressor1.setFilePath_OutputValues("src/input_refrigeration/output_comp1.csv");
		compressor1.setEnergyCarrierOutput(COLDWATER);
		compressor1.setFilePath_SystemStates("src/input_refrigeration/states_comp1.csv");
		SystemParameterExtraction.getFilePathResourceDataList().add(compressor1);

		TimeSeriesData compressor2 = new TimeSeriesData();
		compressor2.setNameOfResource("RefrigerationMachine1");
		compressor2.setFilePath_TimeStamps("src/input_refrigeration/timestamps.csv");

		List<String> energyCarrierInputComp2 = new ArrayList<String>(); 
		energyCarrierInputComp2.add(ELECTRICITY);
		compressor2.setEnergyCarriersInput(energyCarrierInputComp2);
		compressor2.setFilePath_InputValues("src/input_refrigeration/inp_comp2.csv");

		compressor2.setFilePath_OutputValues("src/input_refrigeration/output_comp2.csv");
		compressor2.setEnergyCarrierOutput(COLDWATER);
		compressor2.setFilePath_SystemStates("src/input_refrigeration/states_comp2.csv");
		SystemParameterExtraction.getFilePathResourceDataList().add(compressor2);

		return listOfFilePaths; 
	}
}
