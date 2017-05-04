#!/usr/bin/env python
# -*- coding: utf-8 -*-

import re
import csv

def le_arquivo(nome_log):
  """le os dados da comunicação armazenados no arquivo de nome 'nome_log'"""
  #compila padrões de regex para ler o log
  le_num_scans = re.compile('\w+\s(\d+)\s')
  le_dados_laser=re.compile(r'## FLASER'+'\s(-?\d+\.\d+)+\s?') 
  #abre o arquivo de log para leitura
  f=open(nome_log,'r')
  #cria tabela de dados coletados
  dados=[]
  #itera pelas linhas do arquivo para o processamento
  for linha in f:
    #ignore as linhas de preâmbulo
    if linha[0:3]!="## ":#não é parte do preâmbulo
      #acha correspondencias para o numero de scans
      m=le_num_scans.match(linha)      
      if m:
        #lê número de scans realizados
        num_scans=int(m.group(1))
        #lê dados obtidos durante os scans pelo LRF
        m=le_dados_laser.findall(linha)#cada lista é compostas de sextetos ordenadas x y theta odom_x odom_y odom_theta
        #adiciona à tabela
        dados.append([float(e) for e in m[0:-4]])
  f.close()
  return dados
  
def grava_csv(dados,nome_arq='dados.csv'):
  """recebe uma lista de listas e grava como um csv em que cada sub-lista corresponde a uma linha"""
  f=open(nome_arq,'a')
  arqCSV=csv.writer(f,delimiter=',',lineterminator='\n')
  for lst in dados:
    arqCSV.writerow(lst)
  f.close()

def converte_log_csv(nome_arq_log,nome_arq_csv='dados.csv'):
  """recebe um arquivo de log CARMEN e retorna um csv"""
  #lê arquivo de log
  dados=le_arquivo(nome_arq_log)
  #prepara campos
  f=open(nome_arq_csv,'w+')
  #campos='x,y,theta,odom_x,odom_y,odom_theta\n'
  #f.write(campos)
  f.close()
  #salva dados no novo arquivo
  grava_csv(dados,nome_arq_csv)