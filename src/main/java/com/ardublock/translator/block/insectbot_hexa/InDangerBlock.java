package com.ardublock.translator.block.insectbot_hexa;

import com.ardublock.translator.Translator;
import com.ardublock.translator.block.TranslatorBlock;
import com.ardublock.translator.block.exception.SocketNullException;
import com.ardublock.translator.block.exception.SubroutineNotDeclaredException;

public class InDangerBlock extends TranslatorBlock
{

	public InDangerBlock(Long blockId, Translator translator, String codePrefix, String codeSuffix, String label)
	{
		super(blockId, translator, codePrefix, codeSuffix, label);
	}

	@Override
	public String toCode() throws SocketNullException, SubroutineNotDeclaredException
	{
		InsectBotHexaUtil.setupEnv(translator);
		
		String ret = "insect.isInDanger()";
		
		return codePrefix + ret + codeSuffix;
	}

}
