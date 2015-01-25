package com.ardublock.translator.block.insectbot_hexa;

import com.ardublock.translator.Translator;

public class InsectBotHexaUtil
{
	public static void setupEnv(Translator translator)
	{
		translator.addHeaderFile("InsectBotHexa.h");
		translator.addHeaderFile("Servo.h");
		
		translator.addDefinitionCommand("InsectBotHexa insect;");
	}
}
