package modelGeneration;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.FileFilter;
import java.util.Comparator;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map.Entry;

import designpatterns.DesignPatterns;
import designpatterns.OptimizationResults;
import designpatterns.ResourceParameters;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;
import systemParameterExtraction.ReadParametersFromDataModel;
import systemParameterModel.Dependency;
import systemParameterModel.SystemParameters;
import validation.DecanterDataset;

/**
 * The Class OptimizationModel.
 */
public class OptimizationModelRefrigeration {
	/** The nolimit. */
	static final double NOLIMIT = 9999;

	/** The Constant INPUT. */
	static final String INPUT = "Input";

	/** The Constant OUTPUT. */
	static final String OUTPUT = "Output";

	/** The Constant SOC. */
	static final String SOC = "SOC";

	/** The Constant POWER. */
	static final String POWER = "Power";

	/** The Constant BINARY. */
	static final String BINARY = "Binary";

	/** The Constant SEGMENT. */
	static final String SEGMENT = "Segment";

	/** The Constant STATE. */
	static final String STATE = "State";

	/** The Constant ONLYONE. */
	static final int ONLYONE = -1; 

	/** The startup cost. */
	static double startupCost = 10; 

	/** The const hydr demand. */
	static double constHydrDemand = 900; 

	/** The Constant SYSTEM_NAME. */
	static final String SYSTEM_NAME = "refrigeration";

	/** The global system parameters. */
	static SystemParameters globalSystemParameters = new SystemParameters();

	/**
	 * The main method.
	 *
	 * @param args the arguments
	 */
	public static void main(String[] args)  {
		setOptimizationParameters();
		try {
			parameterizeOptModel();
		} catch (IloException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
			System.err.println("Problem with optimization model");
		}
	}

	/**
	 * Sets the optimization parameters, primarily in ArrayList<ResourceParameters> resourceParameters.
	 */
	public static void setOptimizationParameters () {

		SystemParameters systemParameters = new SystemParameters();
		String filePath = "src/input_"+SYSTEM_NAME+"/"; 
		filePath = "src/output/";
//		filePath +="systemParameters_2024-08-28_14-30-26"; 
		filePath += "systemParameters_2024-08-28_14-30-26_no-target";
		filePath += ".json";
		try {
			systemParameters = ReadParametersFromDataModel.readJson(filePath);
		} catch (Exception e) {
			ModelGenerationDecanter.deriveSystemParameters();
			filePath =  "src/input_"+SYSTEM_NAME;
			filePath += "/"+getMostRecentFile(filePath); 
			systemParameters = ReadParametersFromDataModel.readJson(filePath);
		}

		if (systemParameters == null) System.err.println("SystemParameters empty");

		setGlobalSystemParameters(systemParameters);

		for (int resource = 0; resource < systemParameters.getResourceParameters().size(); resource++) {
			ResourceParameters resourceParameters = new ResourceParameters();
			resourceParameters = systemParameters.getResourceParameters().get(resource);
			resourceParameters.setNumberOfSystemStates(resourceParameters.getSystemStates().size());
			DesignPatterns.getResourceParameters().add(resourceParameters);
		}

		DesignPatterns.setOptimalityGap(0.001); // default 10e-4 = 0.001
		DesignPatterns.setTimeInterval(systemParameters.getTemporalResolutionOptimizationModel());

				DesignPatterns.setArrayLength((int) (10/systemParameters.getTemporalResolutionOptimizationModel())); // set arrayLength in # of time steps
//		DesignPatterns.setArrayLength(40);
	}


	/**
	 * Parameterize opt model.
	 *
	 * @throws IloException the ilo exception
	 */
	public static void parameterizeOptModel () throws IloException {
		String nameOfModel = "OptModel_"+SYSTEM_NAME;
		try {

			//-------------------------------------------------------------------- Create Resource Decision Variables --------------------------------------------------------------------
			DesignPatterns.creationOfDecisionVariables_Names(-1);
			// ------------------------------------------------------------------------ Use of Design Patterns--------------------------------------------------------------------


			// Parameterize resource models
			for (ResourceParameters resourceParameters : getGlobalSystemParameters().getResourceParameters()) {
				String nameOfResource = resourceParameters.getName();
				if (resourceParameters.isStorage() == false  && resourceParameters.isSecondaryResource()==false) {
					DesignPatterns.generateInputOutputRelationship(nameOfResource);
				} else {
					DesignPatterns.generateEnergyBalanceForStorageSystem(nameOfResource);
				}

				if (!(resourceParameters.getSystemStates().isEmpty())) {
					DesignPatterns.generateSystemStateSelectionByPowerLimits(nameOfResource);
					DesignPatterns.generateStateSequencesAndHoldingDuration(nameOfResource);
					DesignPatterns.generateRampLimits(nameOfResource, INPUT);
				}
			}


			// Set up and add dependencies
			setUpDependencies(); 

//			RefrigerationMachine1-Output-Power
//			RefrigerationMachine2-Output-Power
			
			IloNumExpr variableSum = DesignPatterns.getCplex().numExpr();
			for (int timeStep = 0; timeStep < DesignPatterns.getArrayLength(); timeStep++) {
				variableSum = DesignPatterns.getCplex().sum(
						variableSum, 
						DesignPatterns.getCplex().prod(
								DesignPatterns.getTimeInterval(), 
								// decVar
								DesignPatterns.getCplex().sum(
										DesignPatterns.getDecisionVariablesVector().get("RefrigerationMachine1-Output-Power")[timeStep],
										DesignPatterns.getDecisionVariablesVector().get("RefrigerationMachine2-Output-Power")[timeStep]
												)
								)
						);
			}
			DesignPatterns.getCplex().addEq(variableSum, 12000);
			
			
			// set objective function 
			IloLinearNumExpr objective = DesignPatterns.getCplex().linearNumExpr();
			for (int i = 0; i < DesignPatterns.getArrayLength(); i++) {
				objective.addTerm(
						DesignPatterns.getTimeInterval()*0.001*DesignPatterns.getElectricityPriceWithOtherInterval(getGlobalSystemParameters().getTemporalResolutionOptimizationModel())[i], 
//						DesignPatterns.getDecisionVariablesVector().get("System-Input--1Power")[i]
								DesignPatterns.getDecisionVariablesVector().get("System-Input-0Power")[i]
						);
			}
			DesignPatterns.getCplex().addMinimize(objective);

			DesignPatterns.getCplex().exportModel("src/output_"+SYSTEM_NAME+"/"+nameOfModel+"_"+getNow()+".lp");

			// solver specific parameters
			//cplex.setParam(IloCplex.Param.Emphasis.Numerical, true);
			DesignPatterns.getCplex().setParam(IloCplex.Param.MIP.Tolerances.MIPGap, DesignPatterns.getOptimalityGap());

			long start = System.currentTimeMillis();
			System.out.println("cplex solve");
			if (DesignPatterns.getCplex().solve()) {
				long end = System.currentTimeMillis();
				long solvingTime = 	(end - start);

				System.out.println("obj = "+DesignPatterns.getCplex().getObjValue());
				System.out.println("solvingTime in ms = "+solvingTime);
				System.out.println(DesignPatterns.getCplex().getCplexStatus());

				List<OptimizationResults> optimizationResults = saveResults();
				String filePath = "src/output_"+SYSTEM_NAME+"/";
				writeResultsFromListToFile(optimizationResults, nameOfModel, filePath);

			} else {
				System.out.println("Model not solved");
			}
		}

		catch (IloException exc) {
			exc.printStackTrace();
		}
		finally {
			if (DesignPatterns.getCplex()!=null)  {
				DesignPatterns.getCplex().close();
				DesignPatterns.globalCplex=null;
			}
		}
	}

	/**
	 * Sets the up dependencies.
	 *
	 * @throws IloException the ilo exception
	 */
	public static void setUpDependencies() throws IloException {
		int depCounter = 0; 
		for (Dependency dependency: getGlobalSystemParameters().getDependencies()) {
			System.out.println("Dependency"+ depCounter);

			List<IloNumVar[]> inputDecVar = new ArrayList<IloNumVar[]>(); 
			List<IloNumVar[]> outputDecVar = new ArrayList<IloNumVar[]>(); 

			for (int inputCounter = 0; inputCounter < dependency.getRelevantInputs().size(); inputCounter++) {
				String nameOfInput = dependency.getRelevantInputsExtended().get(inputCounter).getResourceName();
				String stateTypeOfInput = dependency.getRelevantInputsExtended().get(inputCounter).getState();
				//				int numberOfInput = -1; 
				System.out.println("input: " + nameOfInput+ " " + stateTypeOfInput);

				if (nameOfInput.contains("SystemOutput")){

					//numberOfInput = Integer.parseInt(nameOfInput.substring(nameOfInput.indexOf("-")));
					nameOfInput = "System";
					IloNumVar[] input;
					double lb; 
					double ub;
					try {
						if (stateTypeOfInput.equals(getGlobalSystemParameters().getOutputSystem())) {
							lb = getGlobalSystemParameters().getMinPowerSystemOutput();
							ub = getGlobalSystemParameters().getMaxPowerSystemOutput();						
						}
						else {
							lb = 0; 
							ub = Double.MAX_VALUE;
						}
					}
					catch (Exception e) {
						lb = 0; 
						ub = Double.MAX_VALUE;
					} 

					input = DesignPatterns.getCplex().numVarArray(
							DesignPatterns.getArrayLength(),  
							lb,
							ub
							);
					DesignPatterns.getDecisionVariablesVector().put("System"+"-"+OUTPUT+"-"+Integer.toString(inputCounter)+POWER, input);
					inputDecVar.add(input);
				} else {
					int numResoureInput = -1; 
					boolean foundResInput = false; 
					boolean resourceIsStorage = false; 
					if (foundResInput == false) {
						for (ResourceParameters resPara: getGlobalSystemParameters().getResourceParameters()) {
							if (resPara.getName().equals(nameOfInput)) {
								for (int ecCounter=0; ecCounter < resPara.getEnergyCarrierInputs().size(); ecCounter++) {
									if (resPara.getEnergyCarrierInputs().get(ecCounter).equals(stateTypeOfInput)) {
										numResoureInput = ecCounter;
										foundResInput = true; 
										resourceIsStorage = resPara.isStorage(); 
									}
								}
							}
						}
					}
					IloNumVar[] input; 
					if (resourceIsStorage == false) {
						input = DesignPatterns.getDecisionVariableFromVector(nameOfInput, INPUT, numResoureInput, POWER);
						System.out.println("Name of Input DecVar: " + nameOfInput + INPUT + numResoureInput + POWER);
					} 
					else {
						input = DesignPatterns.getDecisionVariableFromVector(nameOfInput, INPUT, -1, POWER);
						System.out.println("Name of Input DecVar: " + nameOfInput + INPUT + -1 + POWER);

					}
					inputDecVar.add(input);
				}
			}

			for (int outputCounter = 0; outputCounter < dependency.getRelevantOutputs().size(); outputCounter++) {
				String nameOfOutput = dependency.getRelevantOutputsExtended().get(outputCounter).getResourceName();
				String stateFlowTypeOutput = dependency.getRelevantOutputsExtended().get(outputCounter).getState();
				System.out.println("output: " + nameOfOutput+ " "+ stateFlowTypeOutput);
				int numberOfOutput = -1; 
				if (nameOfOutput.contains("SystemInput")){
					//					numberOfOutput = Integer.parseInt(nameOfOutput.substring(nameOfOutput.indexOf("-")));
					nameOfOutput = "System";
					IloNumVar[] output;

					boolean foundSysInput = false; 
					if (foundSysInput == false) {
						for (int i = 0; i<getGlobalSystemParameters().getInputsSystem().size(); i++) {
							if (getGlobalSystemParameters().getInputsSystem().get(i).equals(stateFlowTypeOutput)) {
								numberOfOutput = i;
								foundSysInput = true; 
							};
						}
					}
					double lb; 
					double ub;
					try {
						if (stateFlowTypeOutput.equals(getGlobalSystemParameters().getInputsSystem().get(numberOfOutput))) {
							lb = getGlobalSystemParameters().getMinPowerSystemInput().get(numberOfOutput);
							ub = getGlobalSystemParameters().getMaxPowerSystemInput().get(numberOfOutput);						
						} else {
							lb = 0; 
							ub = Double.MAX_VALUE;
						}
					} catch (Exception e) {
						lb = 0; 
						ub = Double.MAX_VALUE;
					}


					output = DesignPatterns.getCplex().numVarArray(
							DesignPatterns.getArrayLength(),  
							lb,
							ub);
					DesignPatterns.getDecisionVariablesVector().put("System"+"-"+INPUT+"-"+Integer.toString(numberOfOutput)+POWER, output);
					System.out.println("Name of Output DecVar: " + "System"+"-"+INPUT+"-"+Integer.toString(numberOfOutput)+POWER);
					outputDecVar.add(output);
				} else {
					IloNumVar[] output = DesignPatterns.getDecisionVariableFromVector(nameOfOutput, OUTPUT, ONLYONE, POWER);
					outputDecVar.add(output);
				}
			}

			IloNumVar[][] inputVariablesDependency =  convertListToArray(inputDecVar); 
			IloNumVar[][] outputVariablesDependency = convertListToArray(outputDecVar);

			if (dependency.getTypeOfDependency().equals("correlative")) {
				DesignPatterns.generateCorrelativeDependency(outputVariablesDependency, inputVariablesDependency);
			} else {
				// dependency.getTypeOfDependency().equals("restrictive")
				IloIntVar[][][] restrictiveDependency	= DesignPatterns.generateRestrictiveDependency(outputVariablesDependency, inputVariablesDependency);
			}
			depCounter++; 
		}
	}

	/**
	 * Generate time series data set.
	 *
	 * @return the decanter dataset
	 */
	public static DecanterDataset generateTimeSeriesDataSet () {
		String[] filePath = new String[2]; 
		filePath[0] = "src/input_decanter/input_tsd_dec1.csv";
		filePath[1] = "src/input_decanter/input_tsd_dec1.csv"; 
		filePath[2] =  "src/input_decanter/input_tsd_dec1.csv"; 

		return importDataSet(filePath);
	}


	/**
	 * Gets the container exchange time steps.
	 * @param arrayLength 
	 *
	 * @return the container exchange
	 */
	public static List<Double> getContainerExchange (int arrayLength) {
		String filePath = ""; 
		List<Double> contExch = new ArrayList<Double>(); 
		double var = 0; 
		for (int timeStep=0; timeStep < arrayLength; timeStep++) {
			contExch.add(timeStep, var);
		}
		int ts = 10; 
		contExch.set(ts, 30000.0);
		return contExch;
	}

	private static DecanterDataset importDataSet (String[] filePath) {
		DecanterDataset dataset = new DecanterDataset(); 

		dataset.setPowerInput(importTSD(filePath[0]));

		dataset.setSludgeDensity(importTSD(filePath[1]));

		dataset.setSludgeInFlow(importTSD(filePath[2]));

		return dataset; 
	}

	/**
	 * Save results.
	 *
	 * @return the list
	 */
	public static List<OptimizationResults> saveResults () {
		List<OptimizationResults> optimizationResults = new ArrayList<OptimizationResults>();

		// get all decVars from Vector and save results to List
		for (Entry<String, IloNumVar[]> decisionVariableSet: DesignPatterns.getDecisionVariablesVector().entrySet()) {
			OptimizationResults decVarResults = new OptimizationResults();

			String decisionVariableName = decisionVariableSet.getKey();
			IloNumVar[] decisionVariable = decisionVariableSet.getValue();

			decVarResults.setVariableName(decisionVariableName);
			List<Double> decVarValues = new ArrayList<Double>();

			for (int timeStep = 0; timeStep < decisionVariable.length; timeStep++) {
				try {
					decVarValues.add(timeStep, DesignPatterns.getCplex().getValue(decisionVariable[timeStep]));
				} catch (UnknownObjectException e) {
					System.err.println("Value not found for " + decisionVariableName +" at time step: " + timeStep);
					e.printStackTrace();
				} catch (IloException e) {
					System.err.println("Value not found for " + decisionVariableName +" at time step: " + timeStep);
					e.printStackTrace();
				}
			}
			decVarResults.setOptimizationResults(decVarValues);
			optimizationResults.add(decVarResults);
		}

		// get all decVars from Matrix and save results to List
		for (Entry<String, IloNumVar[][]> decisionVariableSet: DesignPatterns.getDecisionVariablesMatrix().entrySet()) {

			String decisionVariableName = decisionVariableSet.getKey();
			IloNumVar[][] decisionVariable = decisionVariableSet.getValue();


			if (decisionVariableName.contains("State")) {
				System.out.println(decisionVariable[0].length);
				for (int width = 0; width < decisionVariable[0].length; width++) {
					OptimizationResults decVarResults = new OptimizationResults();
					decVarResults.setVariableName(decisionVariableName + "-" + Integer.toString(width));
					List<Double> decVarValues = new ArrayList<Double>();

					for (int timeStep = 0; timeStep < decisionVariable.length; timeStep++) {
						try {
							// state variables defined: statesIntArrayResource[timeStep][state] 
							decVarValues.add(timeStep, DesignPatterns.getCplex().getValue(decisionVariable[timeStep][width]));
						} catch (UnknownObjectException e) {
							System.err.println("Value not found for " + decisionVariableName + "[" + width +"] "+" at time step: " + timeStep);
							decVarValues.add(timeStep, (double) -1);
							e.printStackTrace();
						} catch (IloException e) {
							System.err.println("Value not found for " + decisionVariableName + "[" + width +"] "+" at time step: " + timeStep);
							decVarValues.add(timeStep, (double) -1);
							e.printStackTrace();
						}
					}	
					decVarResults.setOptimizationResults(decVarValues);
					optimizationResults.add(decVarResults);
				}


			} else {
				// other variables defined as [width][timestep]
				for (int width = 0; width < decisionVariable.length; width++) {
					OptimizationResults decVarResults = new OptimizationResults();
					decVarResults.setVariableName(decisionVariableName + "-" + Integer.toString(width));
					List<Double> decVarValues = new ArrayList<Double>();

					for (int timeStep = 0; timeStep < decisionVariable[0].length; timeStep++) {
						try {
							// state variables defined: statesIntArrayResource[timeStep][state] 
							decVarValues.add(timeStep, DesignPatterns.getCplex().getValue(decisionVariable[width][timeStep]));
						} catch (UnknownObjectException e) {
							System.err.println("Value not found for " + decisionVariableName + "[" + width +"] "+" at time step: " + timeStep);
							decVarValues.add(timeStep, (double) -1);
							e.printStackTrace();
						} catch (IloException e) {
							System.err.println("Value not found for " + decisionVariableName + "[" + width +"] "+" at time step: " + timeStep);
							decVarValues.add(timeStep, (double) -1);
							e.printStackTrace();
						}
						decVarResults.setOptimizationResults(decVarValues);
						optimizationResults.add(decVarResults);
					}	
				}
			}

		}
		return optimizationResults;
	}

	/**
	 * Write results to file.
	 *
	 * @param optimizationResults the optimization results
	 * @param fileName the file name
	 * @param filePath the file path
	 */
	public static void writeResultsFromListToFile (List<OptimizationResults> optimizationResults, String fileName, String filePath) {
		// Get the current date and time
		LocalDateTime currentDateTime = LocalDateTime.now();
		// Define the desired date and time format
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
		// Format the current date and time using the formatter
		String formattedDateTime = currentDateTime.format(formatter);


		//TODO check if method works as intended!
		double contentToWrite; 
		try {
			FileWriter myWriter = new FileWriter(filePath+fileName+"_"+formattedDateTime+".csv");
			String header = "timeStamp"; 
			for (int i = 0; i < optimizationResults.size(); i++) {
				header = header+","+ optimizationResults.get(i).getVariableName();
			}
			myWriter.write(header);
			myWriter.write("\n");
			for (int timeStep = 0; timeStep < DesignPatterns.getArrayLength()+1; timeStep++) {
				myWriter.write(Double.toString(timeStep).replace(".", ","));
				for(int resultsCounter = 0; resultsCounter < optimizationResults.size(); resultsCounter++) {
					myWriter.write(";"); // Use semicolon as separator
					//myWriter.write(Double.toString(contentToWrite[i][j]));
					try {
						contentToWrite = optimizationResults.get(resultsCounter).getOptimizationResults().get(timeStep);
					} catch (Exception e) {
						contentToWrite = 0; // TODO: handle exception
					}
					myWriter.write(Double.toString(contentToWrite).replace(".", ",")); // Replace decimal point with comma
				}
				myWriter.write("\n");
			}
			myWriter.close();
			System.out.println("Successfully wrote data to the file "+ fileName+".csv.");
		} catch (IOException e) {
			e.printStackTrace();
		}

	}

	/**
	 * Gets the global system parameters.
	 *
	 * @return the globalSystemParameters
	 */
	public static SystemParameters getGlobalSystemParameters() {
		return globalSystemParameters;
	}

	/**
	 * Sets the global system parameters.
	 *
	 * @param globalSystemParameters the globalSystemParameters to set
	 */
	public static void setGlobalSystemParameters(SystemParameters globalSystemParameters) {
		OptimizationModelRefrigeration.globalSystemParameters = globalSystemParameters;
	}

	/**
	 * Import TSD.
	 *
	 * @param filePath the file path
	 * @return the list
	 */
	private static List<Double> importTSD(String filePath) {
		List<Double> dataList = new ArrayList<>();

		try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
			String line;
			while ((line = reader.readLine()) != null) {
				// Assuming the CSV contains only one column of numerical values
				double value = Double.parseDouble(line.trim());
				dataList.add(value);
			}
		} catch (IOException | NumberFormatException e) {
			e.printStackTrace();
		}
		return dataList; 
		//		// Convert List<Double> to double[]
		//		double[] dataArray = new double[dataList.size()];
		//		for (int i = 0; i < dataList.size(); i++) {
		//			dataArray[i] = dataList.get(i);
		//		}
		//
		//		return dataArray;
	}
	/**
	 * Convert list to array.
	 *
	 * @param listOfArrays the list of arrays
	 * @return the ilo num var[][]
	 */
	public static IloNumVar[][] convertListToArray(List<IloNumVar[]> listOfArrays) {
		int size = listOfArrays.size();
		IloNumVar[][] resultArray = new IloNumVar[size][];

		for (int i = 0; i < size; i++) {
			resultArray[i] = listOfArrays.get(i);
		}

		return resultArray;
	}

	/**
	 * Gets the now.
	 *
	 * @return the now
	 */
	public static String getNow () {
		LocalDateTime currentDateTime = LocalDateTime.now();
		// Define the desired date and time format
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
		// Format the current date and time using the formatter
		String formattedDateTime = currentDateTime.format(formatter);
		return formattedDateTime; 
	}

	public static String getMostRecentFile(String directoryPath) {
		File directory = new File(directoryPath);

		if (!directory.isDirectory()) {
			throw new IllegalArgumentException("The specified path is not a directory");
		}

		// Get all the files in the directory
		File[] files = directory.listFiles(new FileFilter() {
			@Override
			public boolean accept(File file) {
				return file.isFile();
			}
		});

		if (files == null || files.length == 0) {
			return null; // No files in the directory
		}

		// Sort the files by last modified time in descending order
		Arrays.sort(files, new Comparator<File>() {
			@Override
			public int compare(File f1, File f2) {
				return Long.compare(f2.lastModified(), f1.lastModified());
			}
		});

		// Return the most recent file name
		return files[0].getName();
	}
}
