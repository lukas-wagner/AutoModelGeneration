package modelGeneration;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;

import designpatterns.DesignPatterns;
import designpatterns.OptimizationResults;
import designpatterns.ResourceParameters;
import ilog.concert.IloException;
import ilog.concert.IloIntVar;
import ilog.concert.IloLinearNumExpr;
import ilog.concert.IloNumVar;
import ilog.cplex.IloCplex;
import ilog.cplex.IloCplex.UnknownObjectException;
import systemParameterExtraction.ReadParametersFromDataModel;
import systemParameterModel.Dependency;
import systemParameterModel.SystemParameters;

/**
 * The Class OptimizationModel.
 */
public class OptimizationModel {
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
	static final String SYSTEM_NAME = "chp";

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
		filePath += "systemParameters_2024-01-18_13-47-12.json"; 
		try {
			systemParameters = ReadParametersFromDataModel.readJson(filePath);
		} catch (Exception e) {
			ModelGenerationDecanter.deriveSystemParameters();
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
	}


	/**
	 * Parameterize opt model.
	 *
	 * @throws IloException the ilo exception
	 */
	public static void parameterizeOptModel () throws IloException {
		String nameOfModel = "OptModel_"+SYSTEM_NAME;
		try {
			//additional parameters for system
			double maxPowerSystem; 

			try {
				maxPowerSystem = getGlobalSystemParameters().getMaxPowerSystemInput().get(0); 
			} catch (Exception e) {
				maxPowerSystem =  Double.MAX_VALUE;
			}

			//-------------------------------------------------------------------- Decision Variables --------------------------------------------------------------------
			DesignPatterns.creationOfDecisionVariables_Names(maxPowerSystem);

			// ------------------------------------------------------------------------ Use of Design Patterns--------------------------------------------------------------------

			// ------------------------------------------------------------------------ Parameterize Design patterns based on parameter set --------------------------------------------------------------------

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

			// set objective function 
			IloLinearNumExpr objective = DesignPatterns.getCplex().linearNumExpr();
			if (SYSTEM_NAME.equals("chp")) {
				for (int i = 0; i < DesignPatterns.getArrayLength(); i++) {
					objective.addTerm(
							0.001*DesignPatterns.getTimeInterval()*DesignPatterns.getElectricityPriceWithOtherInterval(getGlobalSystemParameters().getTemporalResolutionOptimizationModel())[i], 
							DesignPatterns.getDecisionVariablesVector().get("System"+"-"+OUTPUT+"-"+POWER)[i]
							);
				}
				DesignPatterns.getCplex().addMaximize(objective);
			} else {
				for (int i = 0; i < DesignPatterns.getArrayLength(); i++) {
					objective.addTerm(
							DesignPatterns.getTimeInterval()*DesignPatterns.getElectricityPriceWithOtherInterval(getGlobalSystemParameters().getTemporalResolutionOptimizationModel())[i], 
							DesignPatterns.getDecisionVariableFromVector("System", OUTPUT, ONLYONE, POWER)[i]
							);
				}
				DesignPatterns.getCplex().addMinimize(objective);
			}
			DesignPatterns.getCplex().exportModel("src/output_"+SYSTEM_NAME+"/"+nameOfModel+"_"+getNow()+".lp");
			//			designpatterns.DesignPatterns.getCplex().exportModel("optimizationmodel.mps");
			//			designpatterns.DesignPatterns.getCplex().exportModel("optimizationmodel.sav");
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
				String nameOfInput = dependency.getRelevantInputs().get(inputCounter);
				int numberOfInput = -1; 
				System.out.println("input: " + nameOfInput);
				if (nameOfInput.contains("SystemOutput")){
					numberOfInput = Integer.parseInt(nameOfInput.substring(nameOfInput.indexOf("-")));
					nameOfInput = "System";
					IloNumVar[] input;
					if (numberOfInput == 0) {
						input = DesignPatterns.getDecisionVariableFromVector(nameOfInput, OUTPUT, ONLYONE, POWER);
					}
					else {						
						input = DesignPatterns.getCplex().numVarArray(
								DesignPatterns.getArrayLength(),  
								0,
								Double.MAX_VALUE
								//								getGlobalSystemParameters().getMinPowerSystemInput().get(numberOfInput), 
								//								getGlobalSystemParameters().getMaxPowerSystemInput().get(numberOfInput)
								);
						DesignPatterns.getDecisionVariablesVector().put("System"+"-"+OUTPUT+"-"+Integer.toString(inputCounter)+POWER, input);
					}
					inputDecVar.add(input);
				} else {
					IloNumVar[] input = DesignPatterns.getDecisionVariableFromVector(nameOfInput, INPUT, 0, POWER);
					inputDecVar.add(input);
				}
			}

			for (int outputCounter = 0; outputCounter < dependency.getRelevantOutputs().size(); outputCounter++) {
				String nameOfOutput = dependency.getRelevantOutputs().get(outputCounter);
				System.out.println("output: " + nameOfOutput);
				int numberOfOutput = -1; 
				if (nameOfOutput.contains("SystemInput")){
					numberOfOutput = Integer.parseInt(nameOfOutput.substring(nameOfOutput.indexOf("-")));
					nameOfOutput = "System";
					IloNumVar[] output;
					if (numberOfOutput == 0) {
						//"System", INPUT, ONLYONE,POWER
						output = DesignPatterns.getDecisionVariableFromVector(nameOfOutput, INPUT, ONLYONE, POWER);
					} else {
						output = DesignPatterns.getCplex().numVarArray(
								DesignPatterns.getArrayLength(),  
								0,
								getGlobalSystemParameters().getMaxPowerSystemOutput());
						DesignPatterns.getDecisionVariablesVector().put("System"+"-"+INPUT+"-"+Integer.toString(outputCounter)+POWER, output);
					}
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
			for (int timeStep = 0; timeStep < DesignPatterns.getArrayLength(); timeStep++) {
				myWriter.write(Double.toString(timeStep).replace(".", ","));
				for(int resultsCounter = 0; resultsCounter < optimizationResults.size(); resultsCounter++) {
					myWriter.write(";"); // Use semicolon as separator
					//myWriter.write(Double.toString(contentToWrite[i][j]));
					contentToWrite = optimizationResults.get(resultsCounter).getOptimizationResults().get(timeStep);
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
		OptimizationModel.globalSystemParameters = globalSystemParameters;
	}

	/**
	 * Import TSD.
	 *
	 * @param filePath the file path
	 * @return the double[]
	 */
	private static double[] importTSD(String filePath) {
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

		// Convert List<Double> to double[]
		double[] dataArray = new double[dataList.size()];
		for (int i = 0; i < dataList.size(); i++) {
			dataArray[i] = dataList.get(i);
		}

		return dataArray;
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
	
}
