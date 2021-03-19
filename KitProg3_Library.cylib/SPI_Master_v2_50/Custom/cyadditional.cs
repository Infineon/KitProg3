/*******************************************************************************
* Copyright 2008-2011, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
********************************************************************************/

using System;
using System.Collections.Generic;
using System.Text;
using System.Reflection;
using System.ComponentModel;

namespace SPI_Master_v2_50
{
    static class CyEnumConverter
    {
        /// <summary>
        /// Converts enum description to value
        /// </summary>
        /// <param name="value"></param>
        /// <param name="_enumType"></param>
        /// <returns></returns>
        public static object GetEnumValue(object value, Type _enumType)
        {
            foreach (FieldInfo fi in _enumType.GetFields())
            {
                DescriptionAttribute dna =
                (DescriptionAttribute)Attribute.GetCustomAttribute(
                fi, typeof(DescriptionAttribute));

                if ((dna != null) && (value.ToString() == (string)dna.Description))
                    return Enum.Parse(_enumType, fi.Name);
            }
            return Enum.Parse(_enumType, value.ToString());
        }

        /// <summary>
        /// Converts enum value to description
        /// </summary>
        /// <param name="value"></param>
        /// <returns></returns>
        public static string GetEnumDesc(object value)
        {
            Type _enumType = value.GetType();
            FieldInfo fi = _enumType.GetField(Enum.GetName(_enumType, value));
            DescriptionAttribute dna =
                (DescriptionAttribute)Attribute.GetCustomAttribute(
                fi, typeof(DescriptionAttribute));

            if (dna != null)
                return dna.Description;
            else
                return value.ToString();
        }

        /// <summary>
        /// Gets all enum descriptions
        /// </summary>
        /// <param name="_enumType"></param>
        /// <returns></returns>
        public static string[] GetEnumDescList(Type _enumType)
        {
            List<string> res = new List<string>();
            foreach (object item in Enum.GetValues(_enumType))
            {
                res.Add(GetEnumDesc(item));
            }
            return res.ToArray();
        }
    }
}
